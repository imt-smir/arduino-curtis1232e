# Comunicação entre o Arduino e o driver Curtis 1232e utilizando o protocolo CanOpen
## Ligação entre o Módulo CAN e o driver: 

Em um barramento CAN dois ou mais dispositivos são ligados em paralelo em uma rede na qual em seus 2 extremos temos resistores de 120 Ohms. Um desses fios do barramento é chamado de CAN HIGH e o outro de CAN LOW. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem1.png)

Em nosso caso só temos 2 dispositivos em nossa rede, então será necessário colocar esse resistor nos 2, por sorte tanto o Módulo CAN quanto o Driver possuem um resistor de 120 Ohms integrado que pode ser acionado caso seja necessário. 

No Módulo CAN precisamos conectar entre si esses 2 pinos: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem2.png)

Já no caso do Driver temos que conectar entre si os 2 pinos marcados em verde (CAN Term H e CAN Term L) 

Em amarelo temos os pinos CAN H e CAN L 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem3.png)
