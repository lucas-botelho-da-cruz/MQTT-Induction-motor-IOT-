# -*- coding: utf-8 -*-
"""
Created on Thu May 22 20:09:35 2025

@author: lucas
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Jun 24 08:46:19 2024

@author: lucas
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Apr 15 09:48:44 2024

@author: lucas
"""

# This script contains the low insulation routine implemented in the project.
# Implementation of an access list for different ESP32 devices.
# Use of an internal MQTT broker (i.e., this script acts as a client and connects to the MQTT broker running on the same device).
# External MQTT (enables connection to an external MQTT broker for sending data over the internet).

import time
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import json
from scipy import signal
from scipy.integrate import cumtrapz
from sklearn.cluster import DBSCAN
import numpy as np
import pandas as pd
import os

pacotes=300
# Inicializando o vetor com zeros

lista_I ={}
lista_IC ={}

start_time = time.time()
contador=0;

mqtt_broker_configs={
    'HOST':'localhost',
    'PORT':1883,
    'ClIENT_NAME':'client_project_L',
    'KEPPALIVE':120,
    'TOPIC1':'C/#',
    'Topic2':'controleLucas',
    'TOPIC3':'curto/#'
    }

mqtt_broker_configs_externo={
    'HOST':'lamet.ufsj.edu.br',
    'PORT':1883,
    'ClIENT_NAME':'client_project_L',
    'TOPIC':'serverL'
    }

keepalive = 120

#funções de ajuste ADC_ESP32------------------------------------------------------------------------
def Calibracao_porta32(H1):
    return (-2.30859E-018 * H1**5) + (0.0000000000000111224 * H1**4) - (0.0000000000116359 * H1**3) - (0.0000000115244 * H1**2) + (0.0000343088 * H1) + 0.0606636 + (H1 * 3.3 / 4095)
    
    
def Calibracao_porta33(L1):
    return (-2.49484E-018 * L1**5) + (0.0000000000000129574 * L1**4) - (0.0000000000185113 * L1**3) + (0.000000000166949 * L1**2) + (0.0000266294 * L1) + 0.0624038 + (3.3 / 4095) * L1

def Calibracao_porta34(C1):
    return (-2.34172E-018 * C1**5) + (0.0000000000000114518 * C1**4) - (0.0000000000131218 * C1**3) - (0.0000000083976 * C1**2) + (0.0000321778 * C1) + 0.0614747 + (3.3 / 4095) * C1
#-------------------------------------------------------------------------------------------------


#----------------Filtro Butter--------------------------------------------------------------------------
def filto_digital(ordem,tempo_a,corteI,corteS):
    # Definição dos parâmetros do filtro digital
    Nf =ordem   # Ordem do filtro
    fs = 1 / tempo_a  # Frequência de amostragem
    Wn = [corteI / (fs/2) ,corteS /(fs/2) ]  # Frequências de corte normalizadas


    # Criação do filtro passa-faixa Butterworth digital
    b, a = signal.butter(Nf, Wn, btype='bandpass')
    
    return b,a
#----------------------------------------------------------------------------------------------------------

#---------------------------------------------- RMS vibração------------------------------------------
def rms_vibracao(df):
    tempo_a=0.0003125
    fs=1/tempo_a

    #tempo total das aquisições
    tempo=len(df)*tempo_a

    #construção do vetor de tempo
    tempo_vetor=np.arange(0,tempo,tempo/len(df))
    
    # Fator de conversão 
    fator = 1/256*9.808*1000

    #tirar a media do sinal
    df= (df-np.mean(df))* fator

    saida=cumtrapz(df, tempo_vetor, initial=0)

    ordem=4
    corteI=10
    corteS=1000
    velocidade=np.array(saida)
    b,a=filto_digital(ordem,tempo_a,corteI,corteS)
    y_filtered = signal.filtfilt(b, a, velocidade)
    F_V=FFT( y_filtered,tempo_a*len( y_filtered),118,121)
    return np.sqrt(np.mean(np.square(y_filtered))),F_V   

#-----------------------------------------------------------------------------------------------------------

#---------------------------------FFT-----------------------------------------------------------------------
def FFT(df,tempo,f1,f2):
    # Calcular a FFT
    janela= np.hanning(len(df))
    df=df * janela 
    freq = np.fft.fftfreq(len(df),tempo/len(df))
    fft_resultado = np.fft.fft(df)
    fftreal=2*np.abs(fft_resultado/len(df))
    
    
    # Apenas frequências positivas
    indices_positivos = freq >0
    freq_positivas = freq[indices_positivos]
    fft_positivo = fftreal[indices_positivos]
    
    # Filtrar para a faixa de 58 a 59 Hz
    indices_faixa = (freq_positivas >= f1) & (freq_positivas <= f2)
    freq_faixa = freq_positivas[indices_faixa]
    fft_faixa = fft_positivo[indices_faixa]

# Encontrar a frequência com maior amplitude
    indice_maior_amplitude = np.argmax(fft_faixa)
    frequencia_maior_amplitude = freq_faixa[indice_maior_amplitude]
    amplitude_maior = fft_faixa[indice_maior_amplitude]
    return	amplitude_maior
#----------------------------------------------------------------------------------------------------------

#Variavei iniciais para tedecção do clock de descida
clock_decida=[0]*2
contador_clock=0


# Função responsavel por detectar o clock de descida e enviar sinal para ESP32 iniciar o ensaio de curto
def curto(soma):
    global contador_clock,clock_decida
    # print(clock_decida)
    if soma<=1:
        contador_clock=contador_clock+1
        clock_decida.pop(0)
        clock_decida.append(0)
        if contador_clock==2:
            contador_clock=0
    else:
        contador_clock=contador_clock+1
        clock_decida.pop(0)
        clock_decida.append(1)
        if contador_clock==2:
            contador_clock=0
    # print(clock_decida)
    if clock_decida[0]==1 and clock_decida[1]==0:
       mqtt_client.publish(mqtt_broker_configs['Topic2'], "2")
       print('a')
#--------------------------------------------------------------------------------------------------------       


#----------Função RMS-----------------------------------------------------------------------------------
def rms(A,B,C):
    return np.sqrt(np.mean(np.square(A))),np.sqrt(np.mean(np.square(B))),np.sqrt(np.mean(np.square(C)))
#-------------------------------------------------------------------------------------------------------

#---------------------Função responsavel por escursionar os sinais de corrente em 0, chamar as funções de ajuste de cada porta e multiplicar pelo ganho do STC013_20A
def calibracao(df_ex1,df_ex2,df_ex3):
    
    calibrado=[]
    for item in range(len(df_ex1)):
        vetor1=Calibracao_porta32(df_ex1.iloc[item])
        vetor2=Calibracao_porta33(df_ex2.iloc[item])
        vetor3=Calibracao_porta34(df_ex3.iloc[item])
        calibrado.append({'A':vetor1,'B':vetor2,'C':vetor3})
    
    calibrado=pd.DataFrame(calibrado)
    
    calibrado['A']=(calibrado['A']-np.mean(calibrado['A']))*20
    calibrado['B']=(calibrado['B']-np.mean(calibrado['B']))*20
    calibrado['C']=(calibrado['C']-np.mean(calibrado['C']))*20
    
    
   
        
    return calibrado['A'],calibrado['B'],calibrado['C']



#---------------------Função responsavel por escursionar os sinais de corrente em 0, chamar as funções de ajuste de cada porta e multiplicar pelo ganho do STC013_20A
def calibracao_curto(df_ex1,df_ex2,df_ex3,df_ex4):
    
    calibrado=[]
    for item in range(len(df_ex1)):
        vetor1=Calibracao_porta32(df_ex1.iloc[item])
        vetor2=Calibracao_porta32(df_ex2.iloc[item])
        vetor3=Calibracao_porta32(df_ex3.iloc[item])
        vetor4=Calibracao_porta32(df_ex3.iloc[item])
        calibrado.append({'A':vetor1,'B':vetor2,'C':vetor3,'D':vetor4})
    
    calibrado=pd.DataFrame(calibrado)
    
    calibrado['A']=(calibrado['A']-np.mean(calibrado['A']))*20
    calibrado['B']=(calibrado['B']-np.mean(calibrado['B']))*20
    calibrado['C']=(calibrado['C']-np.mean(calibrado['C']))*20
    calibrado['D']=(calibrado['D']-np.mean(calibrado['D']))*20
    
    
   
        
    return calibrado['A'],calibrado['B'],calibrado['C'],calibrado['D']



          
#---------------------------inicializa o Mqtt-------------------------------------------------------
def start():
    
    mqtt_client_connection_externo = MqttClientConnection(
        mqtt_broker_configs_externo['HOST'],
        mqtt_broker_configs_externo['PORT'],
        mqtt_broker_configs_externo['ClIENT_NAME'],
        keepalive)
    
    mqtt_client_connection_externo.start_connection_externo()
    
    mqtt_client_connection= MqttClientConnection(
        mqtt_broker_configs['HOST'],
        mqtt_broker_configs['PORT'],
        mqtt_broker_configs['ClIENT_NAME'],
        mqtt_broker_configs['KEPPALIVE'])
    
    mqtt_client_connection.start_connection()
    agora = time.time()
    while True:
        tempo=time.time()-agora
        if tempo>=10:
            print(1)
            #controle 1 aquisita corrente # controle 2 aquisita vibração #controle 3 curto
            mqtt_client.publish(mqtt_broker_configs['Topic2'], "1")
            agora = time.time()
#-------------------------------------------------------------------------------------------------------           
    
#-----------------------callback de conecções----------------------------------------------------------
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print('me conectei!')
        client.subscribe([(mqtt_broker_configs['TOPIC1'], 0), (mqtt_broker_configs['TOPIC3'], 0)])
       
    else:
        print(f'erro ao me conectar:{rc}')

def on_connect_ex(client, userdata, flags, rc):
    if rc==0:
        print('Me conectei ao servidor!')
    else:
        print(f'erro ao me conectar:{rc}')
#-------------------------------------------------------------------------------------------------------- 

#------------------------------Mensagens de itens subscritos---------------------------------------------      
def on_subscribe(client, userdata, mid, granted_qos):
    print(f'Subscrito nos tópicos: {mqtt_broker_configs["TOPIC1"]}')
    print(f'Subscrito nos tópicos: {mqtt_broker_configs["TOPIC3"]}')



#--------------------------------Call back rotina de curto-----------------------------------------------
#contator da rotina de curto
p=0

#listas e dataframe que sera utilizado na rotina de curto
coleta_curto={'rmsa':[],'rmsb':[],'rmsc':[],'rmsd':[]}
coleta_curto_2={'FA':[],'FB':[],'FC':[]}
coleta_curto_3={'FA':[],'FB':[],'FC':[]}
df1_C=pd.DataFrame()

def callback_curto(client, userdata, message):
    global lista_IC,df_ex_C,df1_C,p,coleta_curto,coleta_curto_2,coleta_curto_3,CC
    payload_C = json.loads(message.payload)
    
    id_esp_C=message.topic[-1]
    #Verifica se o id do esp é novo
    try:
        lista_IC[str(id_esp_C)] is not None
    except KeyError: #Novo id causa nova lista a ser criada para aquele id.
        lista_IC[str(id_esp_C)] = [0]*pacotes
     
    lista_IC[str(id_esp_C)].pop(0)
    lista_IC[str(id_esp_C)].append(payload_C)
    
    
   
   
    #criação do dataframe de dados
    if payload_C['R'][0]==-1:
        df_C=pd.DataFrame(lista_IC[id_esp])
        df_ex_C=pd.DataFrame({
        'A': df_C['A'].explode(),
        'B': df_C['B'].explode(),
        'C': df_C['C'].explode(),
        'Z': df_C['Z'].explode()
    }).reset_index(drop=True)
        df1_C=pd.concat([df1_C,df_ex_C],axis=0)
        A,B,C,D=calibracao_curto(df_ex_C['A'],df_ex_C['B'],df_ex_C['C'],df_ex_C['D'])
        rmsa,rmsb,rmsc,rmsd=rms(A,B,C,D)
        p=p+1

            
        coleta_curto['rmsa'].append(rmsa)
        coleta_curto['rmsb'].append(rmsb)
        coleta_curto['rmsc'].append(rmsc)
        coleta_curto['rmsd'].append(rmsd)

        
        if p==1:
             coleta_curto_2['FA'].append(np.mean([rmsa,rmsb,rmsc,rmsd]))
        if p==2:
             coleta_curto_2['FB'].append(np.mean([rmsa,rmsb,rmsc,rmsd]))

        if p==3:
            coleta_curto_2['FC'].append(np.mean([rmsa,rmsb,rmsc,rmsd]))
           
#Garante a integridade da coleta
            if coleta_curto_2['FA'][0] > 1 and coleta_curto_2['FB'][0] > 1 and coleta_curto_2['FC'][0] > 1:
                coleta_curto_3['FA'].append(coleta_curto_2['FA'])
                coleta_curto_3['FB'].append(coleta_curto_2['FB'])
                coleta_curto_3['FC'].append(coleta_curto_2['FC'])

                if len( coleta_curto_3['FA'])==7:
# O valor 3,7972 corresponde a corrente média no estado normal do motor somada ao seu desvio padrão nos ensaios com posições aleatórios do rotor
                    if abs(np.mean(coleta_curto_3['FB']))>3.7972:
                        CC=1.0
                        mqtt_client_ex.publish(mqtt_broker_configs_externo['TOPIC'],json.dumps([id_esp_C,0.0,0.0,0.0,0.0,0.0,0.0,0.0,CC,0.0]))
                        coleta_curto_3={'FA':[],'FB':[],'FC':[]}
                    else:
                        CC=0.0
            p=0
            coleta_curto_2={'FA':[],'FB':[],'FC':[]}
            coleta_curto={'rmsa':[],'rmsb':[],'rmsc':[],'rmsd':[]}
            df1_C=pd.DataFrame()
#--------------------------------------------------------------------------------------------------------------------------------------------

         
#--------------------------------------------callback leitura dos dispossitivos-------------------------------------------------------------

#Contador utilizado na call back eitura dos dispossitivos , ele permite fazer medias se mudado dentro da função
contador=0
#lista utilizadas dentro da callback
medidas={'rmsa':[],'rmsb':[],'rmsc':[],'FFT1':[],'FFT2':[],'FFT3':[],'VZ':[],'CC':[]}
#dataFrame utilizado dentro da calback
df1=pd.DataFrame()
#variavel responsavel pelo envio de informação em caso de detecção de curto, 1 corresponde a uma condição de curto
CC=0.0

def callback_corrente(client, userdata, message):
    global contador,agora1,lista_I,df,id_esp,df_ex,payload,medidas,df1,CC
    agora1=time.time()
    # Decodifica a mensagem recebida
    payload = json.loads(message.payload)
    
    id_esp=message.topic[-1]
    #Verifica se o id do esp é novo
    try:
        lista_I[str(id_esp)] is not None
    except KeyError: #Novo id causa nova lista a ser criada para aquele id.
        lista_I[str(id_esp)] = [0]*pacotes
     
    lista_I[str(id_esp)].pop(0)
    lista_I[str(id_esp)].append(payload)
    
    
   
   
    #criação do dataframe de dados
    if payload['R'][0]==-1:
        df=pd.DataFrame(lista_I[id_esp])
        df_ex=pd.DataFrame({
        'A': df['A'].explode(),
        'B': df['B'].explode(),
        'C': df['C'].explode(),
        'Z': df['Z'].explode()
    }).reset_index(drop=True)

        df1=pd.concat([df1,df_ex],axis=0)
        A,B,C=calibracao(df_ex['A'],df_ex['B'],df_ex['C'])
        rmsa,rmsb,rmsc=rms(A,B,C)
)

        
        RMSV,FFTV=rms_vibracao(df_ex['Z'])

      

        medidas['rmsa'].append(rmsa)
        medidas['rmsb'].append(rmsb)
        medidas['rmsc'].append(rmsc)
        

        medidas['FFT1'].append(FFT(A,len(A)*0.0005,58,60))
        medidas['FFT2'].append(FFT(B,len(B)*0.0005,58,60))
        medidas['FFT3'].append(FFT(C,len(C)*0.0005,58,60))
        medidas['VZ'].append(RMSV)

            

        
        if(len(medidas['rmsa'])==1):

            
            somac=rmsa+rmsb+rmsc

            medidas['CC'].append(0)
                
            mqtt_client_ex.publish(mqtt_broker_configs_externo['TOPIC'],json.dumps([id_esp,np.mean(medidas['FFT1']),np.mean(medidas['FFT2']),np.mean(medidas['FFT3']),np.mean(medidas['rmsa']),np.mean(medidas['rmsb']),np.mean(medidas['rmsc']),np.mean(medidas['VZ']),CC,FFTV]))
            medidas={'rmsa':[],'rmsb':[],'rmsc':[],'FFT1':[],'FFT2':[],'FFT3':[],'VZ':[],'CC':[]}       
            
        
        # curto(somac)

        contador=0
       
   
  
#---------------------------------------------Funções responsaveis pelo funcionamento do MQTT---------------------------------------------------------
def on_message_ex(client, userdata, message):
    time.sleep(1)

    

class MqttClientConnection:
    def __init__(self, broker_ip: str, port: int, client_name:str, keepalive=60):
        self.__broker_ip=broker_ip
        self.__port=port
        self.__client_name=client_name
        self.__keepalive=keepalive
    
    def start_connection(self):
        global mqtt_client
        mqtt_client=mqtt.Client(self.__client_name)
        #callback
        mqtt_client.on_connect=on_connect
        mqtt_client.on_subscribe=on_subscribe
        mqtt_client.message_callback_add("C/#",callback_corrente)
        mqtt_client.message_callback_add("curto/#",callback_curto)
        
        mqtt_client.connect(host=self.__broker_ip, port=self.__port,keepalive=self.__keepalive)
        mqtt_client.loop_start()
    
    def start_connection_externo(self):
        global mqtt_client_ex
        
        mqtt_client_ex=mqtt.Client(self.__client_name)
        #callback
        mqtt_client_ex.on_connect=on_connect_ex
        mqtt_client_ex.on_subscribe=on_subscribe
        mqtt_client_ex.on_message= on_message_ex
        
        
        mqtt_client_ex.connect(host=self.__broker_ip, port=self.__port,keepalive=self.__keepalive)
        
        mqtt_client_ex.loop_start()


start()

#--------------------------------------------------------------------------------------------------------------------------




