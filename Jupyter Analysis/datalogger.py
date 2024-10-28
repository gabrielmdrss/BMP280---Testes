from datetime import datetime
import serial

baud = 1000000
porta = 'COM3'
arquivo = "comparacaoDoisSensoresLadoaLado.csv"
cabeçalho = "Data, Hora,Pressao Atmosferica 1,Temperatura 1,Pressao Atmosferica 2,Temperatura 2\n"

ser = serial.Serial(porta, baud)
ser.flushInput()
file = open(arquivo, "a")
file.write(cabeçalho)

while True:

    data_hora_atual = datetime.now()
    hora_formatada = data_hora_atual.strftime("%H:%M, ")
    data_formatada = data_hora_atual.strftime("%Y/%m/%d, ")
    dados = data_formatada + hora_formatada + str(ser.readline().decode("utf-8"))
    file = open(arquivo, "a")
    file.write(dados)
    