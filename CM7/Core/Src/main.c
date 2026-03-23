/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MAX_CHAR 32			//Definição do limite máximo do tamanho da string de entrada
#define MAX_STRING 10		//Definição do limite máximo dos tokens da string de entrada
#define MAX_DELIM 10		//Definição do limite máximo dos caracteres delimitadores
#define MAX_OUT 512			//Definição do tamanho máximo da resposta ao utilizador
#define HAL_MAX_DELAY 1000	//Definição do delay de leitura da usart

#define TIM3_ARR  (64000-1)	// Valor de início do Auto-reload register
#define TIM3_CCR  (32000-1) // Valor de início do Capture-Compare Register

#define VOL 10				//Nº de voltas (máximo valor = 10)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char delim[MAX_DELIM] = " ";										  //Declaração e inicialização dos caracteres delimitadores
char input[MAX_CHAR] = "";												  //Declaração e inicialização do buffer de entrada da uart
volatile char rx_buff[MAX_CHAR];										  //Buffer de receção de caracteres
volatile uint8_t data_ready = 0;										  //Declaração e inicialização da flag de receção da uart
volatile uint8_t ov;													  //Variável para a flag de interrupção genérica (overflow)
volatile int inc_pos=0;													  //Variável para incremento da posição
volatile int inc_vel=0;													  //Variável para incremento da velocidade
volatile int lim=0;														  //Variável para definir o limite das voltas (10)
volatile int vol=0;														  //Variável para voltas
volatile int dir=0;														  //Variável de direção
int CS=0;																  //Variável de estado do Sistema de Controlo (0:Reset, 1:Config, 2:Manual, 3:Auto)
//int last_CS=0;
int EN=0;																  //Variável de ativação (Enable) dos motores (0: Desligado, 1: Ligado)
int R=0;																  //Variável da Leitura (neste caso, termina a amostragem, seja do modo contínuo ou do modo limitado)
int RT=2;																  //Variável de definição de leitura (neste caso definido para leitura da posição e da velocidade)
int laps=0;																  //Nº de voltas
char resposta[MAX_OUT] = {};											  //Resposta ao utilizador

typedef enum{                           // Enumeração dos tipos de parâmetros esperados na análise sintática
    INT = 0,                            // Parâmetro para números inteiros (ex: os endereços de memória para MR/MW)
    DEC,                                // Parâmetro para números decimais (ex: o duty-cycle para o PWM)
    PIN,                                // Parâmetro hexadecimal, representando o mapa dos pinos (ex: o mapa de pinos para PI/PO/RD/WD)
    BIN,                                // Parâmetro hexadecimal, representando os valores lógicos (ex: os valores dos pinos para WD)
    CHAR,                               // Parâmetro caractere (ex: identificação da porta 'A', 'B', 'C', 'D', 'E', 'F' e 'G')
	DIG,								// Dígito único (0-9, sendo usado em comandos como CS ou RT
	FLOAT,								// Valor decimal (ex: 3.4), sendo usado para os ganhos do PID (Kp, Ki, Kd)
	SIGN,								// Sinal de operação (unsigned/signed), usado no comando de PWM
	UINT,								// Valor inteiro de 16 bits (0-65536), usado para o período de amostragem (HW)
}Type;

typedef enum{							//Enumeração dos tipos de erros
	ALL_OK = 0,							//Sem erros
	ERR,								//Erro genérico
	ERR_CMD,							//Erro de comando
	ERR_PAR,							//Erro de parâmetro
	ERR_OVR,							//Erro de overflow
	//ERR_STATE,
}Error;

typedef enum{							//Enumeração dos tipos de comandos a usar
	CMD_HELP = 0,						//Comando Help
	CMD_CS,								//Comando Control System
	CMD_EN,								//Comando Enable dos Motores
	CMD_HW,								//Comando do período de amostragem
	CMD_RT,								//Comando de configuração do tipo de leitura
	CMD_R,								//Comando de leitura
	CMD_PWM,							//Comando do PWM (tensão normalizada)
	CMD_RESPOS,							//Comando de reinicialização da posição 0 do disco
	CMD_PID,							//Comando de configuração de variáveis e parâmetros do controlador PID
}Cmd;

typedef struct {                        // Estrutura gerada pelo analisador léxico/sintático (parser)
    int data[10];                       // Vetor de tokens convertidos em inteiros/dados numéricos
    Error state;                        // Regista se ocorreu algum erro durante o parsing da string
}Tokens;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int print(char string[MAX_OUT]){															//Função de apresentação de uma string no terminal
	return HAL_UART_Transmit(&huart3, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);		//Transmissão pela usart 3 da string desejada
}
int start_scan(char string[MAX_OUT]){														//Função de início da leitura de uma string no terminal
	return HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t*)string, MAX_CHAR);				//Início da receção da uart
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void upperCase(char string[]){					//Função de uniformalização de uma string
	int i=0;									//Declaração e inicialização do iterador
	int len = strlen(string);					//Declaração e inicialização do tamanho da string
	for(i=0; i<len; i++){						//Ciclo de verificação de letra maiúscula da string toda
		if('a'<=string[i] && string[i]<='z'){	//Verifica se o caractere é uma letra minúscula
			string[i]-=32;						//Garante que o caractere é maiúsculo
		}
	}

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int pins[16]={}; 													//Declaração e inicialização do array de pinos
int n=0; 															//Declaração e inicialização do índice do array

Error validate(char par[], int digits, int type, int *out, int hex){ 		//Função para validar os parâmetros implementados pelo utilizador
	int i = 0; 														//Declaração e inicialização do iterador
	uint32_t num = 1; 												//Declaração e inicialização do inteiro convertido da string de entrada
	int max = 10; 													//Declaração e inicialização da variável de tamanho máximo do parâmetro de entrada
	//char temp[64] = {}; 											//Declaração e inicialização de uma string temporária
	if((type != FLOAT) && (type != CHAR) && (type != SIGN)){		//Verficação do tipo de parâmetro para ver se é preciso verificação do tamanho
		for(i=0; i<digits; i++){									//Ciclo para verificação dos dígitos
			if(par[i]=='\0') break;									//Se a string não utilizar todos os caracteres disponíveis, acaba o ciclo
			if(!isxdigit(par[i])) return ERR;						//Se o utilizador escrever algo que não seja um dígito hexadecimal, retorna erro
		}
		max = pow(16,digits);										//Valor máximo do parâmetro
		if(hex){
			num = (uint32_t)strtoul(par, NULL, 16);					//Conversão da string para um valor numérico usando a base 16 (Hexadecimal)
		}else{
			num = (uint32_t)strtoul(par, NULL, 10);					//Conversão da string para um valor numérico usando a base 10 (Decimal)
		}
	}
	if(num<max){													//Se o parâmetro de entrada não ultrapassa o tamanho máximo
		switch (type){												//Diferentes casos
		case INT:													//Parâmetro inteiro genérico (Endereços, Tamanhos)
			*out = num;												//O endereço da string de saída toma o valor de num
			break;
		case DEC:													//Conversão específica para duty-cycle ou valores decimais
			num *= 100;												//Multiplicação para percentagem
			int uni = num / 255;									//Obtenção do valor inteiro (quociente)
			int dec = num % 255;									//Obtenção do valor decimal restante (resto)
			dec*=100;												//Multiplicação para percentagem da parte decimal
			dec/=255;												//Obtenção da percentagem decimal
			*out = (uni*100)+dec+(num*100);							//Armazena o valor formatado em centésimas na string de saída
			break;
		case PIN:													// Mapa de pinos (<pinsMap> em hexadecimal)
			*out = num;												//O endereço da string de saída toma o valor de num
			break;
		case BIN:													// Valores binários (<pinValues> em hexadecimal)
			*out = num;												//O endereço da string de saída toma o valor de num
			break;
		case CHAR:                                          		// Identificador da porta ('A' até 'Z')
		    if(par[1] != '\0') return ERR;                  		// Garante que é apenas um caractere
		    if(par[0] < 'A' || par[0] > 'Z') return ERR;    		// Valida se está no intervalo ASCII de letras maiúsculas
		    *out = par[0];                                  		//Novo valor do endereço da string de saída
		    break;
		case DIG:
			if(num < 10){											// Valida se o número é um dígito único (0 a 9)
				*out = num;											// Atribui o valor convertido à variável de saída
				break;
			}
			return ERR;												// Retorna erro se o parâmetro tiver mais do que um dígito
			break;
		case FLOAT:
			if((par[1]!='.') && (par[1]!=',')) return ERR;				// Verifica se o segundo caractere é um separador decimal válido (. ou ,)
			for(i=0; i<digits; i+=2){									// Ciclo de validação de caracteres (garante que são dígitos antes de converter)
				if(par[i]=='\0') break;									//Se a string não utilizar todos os caracteres disponíveis, acaba o ciclo
				if(!isxdigit(par[i])) return ERR;						//Se o utilizador escrever algo que não seja um dígito, retorna erro
				if((uint32_t)strtoul(par[i], NULL, 16)>9) return ERR;	//Se o utilizador puser um número float cujos dígitos não estejam entre 0 e 9, retorna erro(não dá a conversão para um número unsigned int)
			}
			// Conversão "manual" da string fixa (x.y) para um inteiro representativo
			int uniF = par[0]-'0';										// Converte o caractere da unidade em valor inteiro
			int decF = par[2]-'0';										// Converte o caractere da décima em valor inteiro
			num = uniF*10 + decF;										// Armazena como valor fixo (ex: 3.4 vira 34) para facilitar os cálculos
			*out = num;													// Atribui o valor convertido à variável de saída
			break;
		case SIGN:														// Interpretação do sentido de rotação ou do sinal para o PWM
			switch(par[0]){
			case ' ':													// Espaço tratado como positivo por defeito
				*out = 1;
				break;
			case '+':													// Sinal positivo
				*out = 1;
				break;
			case '-':													// Sinal negativo (sentido inverso)
				*out = 0;
				break;
			default:													// Caractere inválido
				return -1;
				break;
			}
			break;
		case UINT:													// Para parâmetros inteiros genéricos (endereços ou o HW)
			*out = num;												// Atribui diretamente o valor convertido anteriormente
			break;
		default:
			return ERR;                                     		//Se não tivermos nenhum dos casos anteriores, dá erro genérico
		}
		return ALL_OK;												//Se no final do switch estiver tudo bem, retorna OK
	}
	return ERR;														//Se a condição num < max não acontecer, retorna erro
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Tokens identify(char *in[MAX_STRING], int count){								//Função de análise sintática dos tokens
	Tokens out;																	//Variável para ver que comando estamos a usar

	if (strcmp(in[0], "?") == 0) {												////Verificação se é o comando de ajuda
		out.data[0] = CMD_HELP;													//Ativa o comando HELP

		out.state = ALL_OK;														//Confirmação da validação (tudo bem)
		return out;																//Retorna o comando escolhido na string de saída
	}else if (strcmp(in[0], "CS") == 0){										////Verificação se é o comando CS (Control System): Altera o estado da máquina de estados
		out.data[0] = CMD_CS;													//Ativa o comando CONTROL SYSTEM (CS)
		if(count==2){															// Espera 1 parâmetro: o número do estado (0, 1, 2 ou 3)
			char *digitStr = in[1];
			if(validate(digitStr, 1, DIG, &(out.data[1]), 0)){					// Valida se o parâmetro é um dígito único (0-3)
				out.state = ERR_PAR;											// Erro se não for um dígito válido
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "EN") == 0){										////Verificação se é o comando EN (Enable): Ativa ou desativa a atuação dos motores
		out.data[0] = CMD_EN;													//Ativa o comando ENABLE DOS MOTORES (EN)
		if(count==2){															// Espera 1 parâmetro (0 para OFF, 1 para ON)
			char *digitStr = in[1];
			if(validate(digitStr, 1, DIG, &(out.data[1]), 0)){					// Valida se o parâmetro é um dígito único (0 ou 1)
				out.state = ERR_PAR;											// Erro se não for um dígito válido
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "HW") == 0){										////Verificação se é o comando HW (Período de Amostragem): Define o período de amostragem em ms
		out.data[0] = CMD_HW;													//Ativa o comando HW
		if(count==2){															// Espera um valor inteiro de 16 bits (UINT)
			char *digitStr = in[1];
			if(validate(digitStr, 4, UINT, &(out.data[1]), 0)){					// Valida se o parâmetro é um conjunto de 1 até 4 dígitos únicos (0-9)
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "RT") == 0){										////Verificação se é o comando RT (Reading Type): Define o que o sistema deve ler (Posição, Velocidade, etc.)
		out.data[0] = CMD_RT;													//Ativa o comando RT
		if(count==2){															// Espera 1 parâmetro de configuração (0 para ler posição, 1 para ler velocidade e 2 para ler ambos)
			char *digitStr = in[1];
			if(validate(digitStr, 1, DIG, &(out.data[1]), 0)){					// Valida se o parâmetro é um dígito único (0, 1 ou 2)
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "R") == 0){											////Verificação se é o comando R (Leitura do ADC)
		out.data[0] = CMD_R;													//Ativa o comando R
		if((count==3) || (count==2)){											// Verifica se o utilizador escreveu exatamente 2 parâmetros (Total de 3 palavras)
			char *digitStr = in[1];												// Apontador para a string do primeiro parâmetro (Canal do ADC)
			char *unitStr = in[2];												// Apontador para a string do segundo parâmetro (Número de amostras ou valor)

			// Valida o primeiro parâmetro como um dígito (DIG) e o segundo como um inteiro sem sinal (UINT)
			if((validate(digitStr, 1, DIG, &(out.data[1]), 0))
					||((out.data[1]==2)&&(validate(unitStr, 4, UINT, &(out.data[2]), 0)))){
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "PWM") == 0){										////Verificação se é o comando PWM (controlo do motor)
		out.data[0] = CMD_PWM;													//Ativa o comando PWM
		if(count==2){															// Espera 1 parâmetro (o valor do PWM com ou sem sinal, ex: +50 ou -50) É 2 porque o sinal é um caractere
			char signStr[2];													// Buffer para armazenar o sinal (+, -, ou vazio)
			char digitStr[4];													// Buffer para armazenar o valor numérico (0-100)

			// Lógica de separação: Se o primeiro caractere não for um número, é um sinal
			if(in[1][0] < '0' || in[1][0] > '9'){
				signStr[0] = in[1][0];											// Captura o sinal (+ ou -)
				digitStr[0] = in[1][1];											// Extrai os dígitos que vêm logo a seguir ao sinal
				digitStr[1] = in[1][2];
				digitStr[2] = in[1][3];
			}else{
				strcpy(signStr," ");											// Se começar logo pelo número, assume o sinal positivo (espaço)
				strcpy(digitStr,in[1]);											// A string numérica é o parâmetro completo
			}

			// Valida o sinal (tipo SIGN) e o valor numérico (tipo UINT)
			if((validate(signStr, 1, SIGN, &(out.data[1]), 0))
					||(validate(digitStr, 3, UINT, &(out.data[2]), 0))){
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;

	}else if (strcmp(in[0], "RESPOS") == 0){									////Verificação se é o comando RESPOS (Reset da posição 0 do disco)
		out.data[0] = CMD_RESPOS;												//Ativa o comando RESPOS
		if(count==1){															// Este comando não aceita parâmetros adicionais
			out.state = ALL_OK;													// Estado OK, pois basta o nome do comando
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if (strcmp(in[0], "PID") == 0){										////Verificação se é o comando 'PID' (ajuste das variáveis e dos parâmetros do controlador PID)
		out.data[0] = CMD_PID;													//Ativa o comando PID
		if(count==3){															// Espera 2 parâmetros (ex: PID 0 1.5 -> Kp=1.5)
			char *digitStr = in[1];												// Primeiro parâmetro: �?ndice do ganho (0=P, 1=I, 2=D)
			char *floatStr = in[2];												// Segundo parâmetro: Valor decimal do ganho

			// Valida o índice como dígito (DIG) e o valor como número de vírgula flutuante (FLOAT)
			if((validate(digitStr, 1, DIG, &(out.data[1]), 0))
					||(validate(floatStr, 3, FLOAT, &(out.data[2]), 0))){
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída
	}
	out.state = ERR_CMD;														// Se não coincidir com nenhum comando conhecido, dá erro de comando
	return out;																	//Retorna o erro de comando na string de saída
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Error check_state(int states[4]){					//Verifica se o estado atual do sistema (CS) consta na lista de estados permitidos para um determinado comando
	int i=0;										// Inicializa a variável auxiliar para percorrer o array
	for(i=0;i<4;i++){								// Ciclo para comparar o estado atual com a lista de estados autorizados

		// Se o estado atual (CS) coincidir com um dos estados da lista
		if(CS==states[i]) return ALL_OK;			 // Validação bem-sucedida: o comando pode ser executado
	}
	// Se o ciclo terminar sem encontrar uma correspondência
	return ERR;										// Retorna erro: o sistema não está num estado que permita este comando
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Tokens parse(char in[MAX_CHAR], const char delim[MAX_DELIM]){		//Função de tokenização da string de entrada
	int count = 0;							  						//Declaração e inicialização do contador do número de tokens encontrados
	char *strings[MAX_STRING]={};									// Vetor de apontadores para guardar cada token

	// Limpeza da string
	char *end = strchr(in, '\n');									// Procura o caractere de nova linha (Line Feed)
	if(end){														//Se estivermos no último caractere da string
		*end = '\0';												// Substitui o mesmo último caractere pelo terminador nulo para limpar a string
	}
	char *end2 = strchr(in, '\r');									// Procura o caractere de Carriage Return (CR) do terminal
	if(end2){														//Se estivermos no último caractere da string
		*end2 = '\0';												// Substitui o mesmo último caractere pelo terminador nulo para limpar a string
	}

	//Tokenização (divisão da string em tokens)
	 char *token = strtok(in, delim);								// Extrai o primeiro token antes do primeiro delimitador
	 while(token!=NULL){											// Enquanto encontrar mais palavras (tokens) na string
		 if(count >= MAX_STRING){									// Prevenção de buffer overflow de tokens
			 //print("ERRO: Limite de tokens atingido");			//Apresenta uma mensagem de erro
			 Tokens err;											//Variável para usar os estados da estrutura gerada pelo analisador léxico/sintático (parser)
			 err.state = ERR_OVR;									// Retorna erro se o comando tiver demasiadas partes
			 return err;											//Encerra o funcionamento do parse
		 }
	 	strings[count++] = token;									// Guarda o apontador do token e incrementa a contagem
	 	token = strtok(NULL, delim);								// O NULL indica ao strtok para continuar na string original
	 }
	 return identify(strings, count);								// Passa o vetor resultante para a Análise Sintática (identify)
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void enable(int e){
	EN=e;
	if(EN){								// Se o comando for para ativar o motor (EN = 1)
		// Inicia a geração de sinais PWM nos canais do Timer 3 para controlar a Ponte-H
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_Base_Start_IT(&htim6);		//início do timer para o PWM
		HAL_GPIO_WritePin(GPIOB, ENABLE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, ENABLE2_Pin, GPIO_PIN_SET);

	}else{								// Se o comando for para desativar o motor (EN = 0)
		// Interrompe imediatamente os sinais PWM para parar a alimentação dos motores
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_TIM_Base_Stop_IT(&htim6);		//início do timer para o PWM
		HAL_GPIO_WritePin(GPIOB, ENABLE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, ENABLE2_Pin,GPIO_PIN_RESET);
		vol = 0;
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void execute(Tokens in){ //Função execute
	// Buffers locais para montagem das mensagens de resposta ao utilizador
	char resposta[MAX_OUT] = {};						//Resposta ao utilizador
	char active[64]={};									// Guarda a lista de pinos formatada (ex: "0, 1 e 2")
	char value[64]={};									// Guarda a lista de valores lógicos formatada
	char out[MAX_OUT]={};								// Buffer auxiliar para concatenação de strings longas (diferente do out referido no código anterior)
	char temp[64]={};									// String temporária para conversões rápidas
	int i=0;											//Declaração e inicialização do iterador
	switch(in.state){									// Primeiro nível de decisão: Verifica se o comando é válido antes de tentar executar
	case ALL_OK:
		switch(in.data[0]){								// Segundo nível de decisão: Identifica qual o comando específico que foi solicitado

		case CMD_HELP:									// Comando '?' - Lista de ajuda
			print("Comandos disponiveis:\r\n");
			print("?                   - Fornece uma lista dos comandos validos.\r\n");
			print("CS <dig>            - Define o estado: 0-Reset, 1-Config, 2-Manual, 3-Auto\r\n");
			print("EN <dig>            - Ativa (1) ou Desativa (0) os motores\r\n");
			print("HW <uint>           - Define periodo de amostragem (1 a 1000 ms)\r\n");
			print("RT <dig>            - Tipo de leitura: 0-Pos, 1-Vel, 2-Ambos\r\n");
			print("R <dig> <uint>      - Leitura: 0-Para, 1-Continua, 2-Limitada (n amostras)\r\n");
			print("PWM <signval>       - Tensão Normalizada (-100 a +100%)\r\n");
			print("RESPOS              - Reinicializa a posicao zero do disco\r\n");
			print("PID <dig> <float>   - Configura PID: 0-yr, 1-Kp, 2-Ki, 3-Kd, 4-alpha, 5-Ler\r\n");
			print("Teclas '/' e '\\'    - Ajuste rapido de PWM (+/- 5 unidades)\r\n");
			break;
		case CMD_CS:									// Execução da mudança de estado
			snprintf(resposta, MAX_OUT,					// Prepara a string de resposta informativa para o utilizador
					"\r\nDefine o estado como %d", in.data[1]);
			print(resposta);							// Envia a confirmação do comando para o terminal série (UART)

			if(in.data[1]<=3){ 							// Valida se o estado solicitado está no intervalo permitido (0 a 3)

				// Implementação das regras de transição da Máquina de Estados (Segurança):
				// Só permite entrar em modo Manual (2) ou Automático (3) se o sistema vier de Configuração (1)
				if((CS==2)&&(in.data[1]!=3)||(CS==3)&&(in.data[1]!=2)||(CS==1)){
					CS = in.data[1];					// Atualiza a variável global que controla o fluxo da máquina de estados
				}else{
					snprintf(resposta, MAX_OUT,			// Prepara uma mensagem de erro se a transição for inválida
							"\r\nERRO DE ESTADO!");		// Bloqueia as transições não permitidas (por exemplo: Manual -> Auto diretamente não dá)
					print(resposta);					// Envia o aviso de erro ao utilizador
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Prepara uma mensagem caso o utilizador insira um número > 3
						"\r\nERRO DE VALOR!");
				print(resposta);						// Informa que o estado solicitado não existe
			}
			break;										// Finaliza a execução do comando CS

		case CMD_EN:									// Execução da ativação do motor (ENABLE)
			snprintf(resposta, MAX_OUT,					// Prepara a string confirmando o valor de enable recebido
					"\r\nDefine o enable como %d", in.data[1]);
			print(resposta);							// Envia a confirmação para a UART
			int en[]={2,3,-1,-1};						// Define a lista de estados permitidos para este comando (Manual e Automático)
			if(!check_state(en)){						// Verifica se o sistema está num estado que permite ligar/desligar motores
				if(in.data[1]<=1){						// Valida se o parâmetro é binário (0: OFF, 1: ON)
					enable(in.data[1]);
					// Formatação da mensagem final indicando o novo estado real dos motores
					snprintf(resposta, MAX_OUT, "\r\nMotores %s", (EN==1) ? "ATIVADOS" : "DESATIVADOS");
					print(resposta);					// Envia o feedback visual para o terminal
				}else{
					snprintf(resposta, MAX_OUT,			// Erro se o valor inserido não for 0 nem 1
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Erro se o sistema estiver num estado em que não dê para usar EN
						"\r\nESTADO ERRADO!");			// Impede uma ativação acidental de motores durante a configuração
				print(resposta);
			}
			break;										// Finaliza a execução do comando EN

		case CMD_HW:									// Execução do comando HW (período de amostragem)
			snprintf(resposta, MAX_OUT,					// Prepara a mensagem de confirmação do parâmetro recebido
					"\r\nDefine o periodo de amostragem como %dms", in.data[1]);
			print(resposta);							// Envia o feedback visual para o utilizador pela UART
			int hw[]={1,-1,-1,-1};						// Define que este comando só é permitido no Estado 1 (Configuração)
			if(!check_state(hw)){						// Verifica se o sistema está no estado de Configuração
				if(in.data[1]<=1000){					// Valida se o período inserido é seguro (limite de 1000ms = 1 segundo)
					__HAL_TIM_SET_AUTORELOAD(&htim6, (in.data[1]-1)?in.data[1]-1:999);
					__HAL_TIM_SET_COUNTER(&htim6, 0);
				}else{
					snprintf(resposta, MAX_OUT,			// Erro se o valor ultrapassar o limite definido por segurança
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Bloqueia a alteração do período se o motor já estiver em Manual ou Automático
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando HW

		case CMD_RT:									// Execução do comando RT (tipo de leitura)
			snprintf(resposta, MAX_OUT,					// Prepara a string de resposta com o modo de leitura pretendido
					"\r\nDefine o modo de leitura como %d", in.data[1]);
			print(resposta);							// Envia confirmação para o terminal
			int rt[]={1,-1,-1,-1};						// Define que este comando só é permitido no Estado 1 (Configuração)
			if(!check_state(rt)){						// Verifica se o sistema está no estado de Configuração
				if(in.data[1]<=2){						// Valida o modo: 0 (Posição), 1 (Velocidade) ou 2 (Ambos)
					RT=in.data[1];						// Guarda o modo de leitura
				}else{
					snprintf(resposta, MAX_OUT,			// Erro se o modo de leitura for inexistente (>2)
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Impede a mudança de tipo de leitura fora do modo de configuração
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando RT

		case CMD_R:										// Execução do comando R
			if(in.data[1]==2){
				snprintf(resposta, MAX_OUT,					// Prepara a string confirmando os parâmetros de leitura recebidos
						"\r\nDefine a leitura como %d com %d leituras", in.data[1], in.data[2]);
				print(resposta);							// Envia o feedback para o terminal série
			}else{
				snprintf(resposta, MAX_OUT,					// Prepara a string confirmando os parâmetros de leitura recebidos
						"\r\nDefine a leitura como %d", in.data[1]);
				print(resposta);							// Envia o feedback para o terminal série
			}

			int r[]={2,-1,-1,-1};						// Define que a leitura manual só é permitida no Estado 2 (Manual)
			if(!check_state(r)){						// Valida se o sistema se encontra no modo Manual
				if(in.data[1]<=2){						// Verifica se o canal/tipo de leitura solicitado é válido (0, 1 ou 2)
					R=in.data[1];						// Guarda-se a leitura que estamos a fazer
					laps=in.data[2];					// Guarda-se o nº de voltas
				}else{
					snprintf(resposta, MAX_OUT,			// Mensagem de erro caso o parâmetro de canal seja inválido
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Bloqueia a leitura se o sistema estiver em Reset, Config ou Auto
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando R

		case CMD_PWM:									//Execução do comando PWM(Controlo da atuação)
			snprintf(resposta, MAX_OUT,					// Mostra o valor do Duty Cycle (%) e o sentido (+ ou -) no terminal
					"\r\nDefine o PWM como %d%% com o direcao %c", in.data[2],(in.data[1])?'+':'-');
			print(resposta);
			int pwm[]={1,2,-1,-1};						// Define que o ajuste do PWM só é válido nos estados 1 (Config) e 2 (Manual)
			if(!check_state(pwm)){						// Verifica se estamos num dos dois estados definidos acima

				if(abs(in.data[2])<=100){				// Valida se o Duty Cycle está entre 0% e 100%

					// Cálculo do valor de comparação (CCR):
					// Converte a percentagem (0-100%) para o valor proporcional ao ARR (Auto-Reload Register) do Timer.
					int pwm=(in.data[2]*__HAL_TIM_GET_AUTORELOAD(&htim3))/100;

					if(in.data[1]) { 					// Lógica para o sentido direto (Forward / +)
						// Define o sinal PWM no Canal 1 e desativa o Canal 2 para rodar num sentido
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   // Canal Direção -
						HAL_Delay(1);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm); // Canal Direção +
						dir=1;
					} else { 							// Lógica para o sentido inverso (Reverse / -)
						// Desativa o Canal 1 e define o sinal PWM no Canal 2 para inverter a polaridade na Ponte-H
					    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);   // Canal Direção +
					    HAL_Delay(1);
					    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm); // Canal Direção -
					    dir=0;
					}
				}else{
					snprintf(resposta, MAX_OUT,			// Erro se o utilizador tentar definir mais de 100% de PWM
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Impede o controlo manual do PWM se o sistema estiver em Modo Automático (Estado 3)
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando PWM

		case CMD_RESPOS:								//Execução do comando RESPOS(Reset da posição 0 do disco)
			snprintf(resposta, MAX_OUT,					// Prepara a mensagem de confirmação para o utilizador
					"\r\nReinicia a posicao");
			print(resposta);							// Envia o feedback para o terminal série
			int respos[]={1,2,-1,-1};					// Define que o reset da posição é permitido em Config (1) ou Manual (2)
			if(!check_state(respos)){					// Verifica se o sistema está num dos estados autorizados
				inc_pos=0;
				inc_vel=0;
			}else{
				snprintf(resposta, MAX_OUT,				// Erro se o utilizador tentar dar reset durante o modo Automático
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando RESPOS

		case CMD_PID:									//Execução do comando PID(Configuração dos ganhos proporcional, integrativo e derivativo)

			// O valor recebido (in.data[2]) está num ponto fixo (valor real * 10).
			// Exemplo: se o utilizador enviou "3.4", o valor armazenado é 34
			uint32_t numP = in.data[2];					// Armazena o valor bruto (ex: 34)
			uint32_t uniP = numP / 10;					// Obtém a parte inteira (ex: 34 / 10 = 3)
			uint32_t decP = numP % 10;					// Obtém a parte decimal (ex: 34 % 10 = 4)

			snprintf(resposta, MAX_OUT,					// Formata a resposta para mostrar o índice do ganho e o valor original com ponto (ex: 3.4)
					"\r\nDefine o PID como %d e posicaoo %d.%d", in.data[1], uniP, decP);
			print(resposta);							//Escrita da resposta

			int pid[]={1,-1,-1,-1};						// A alteração de ganhos só é permitida no Estado 1 (Configuração)
			if(!check_state(pid)){						// Garante a estabilidade do sistema impedindo alterações em pleno voo
				if(abs(in.data[1])<=5){					// Valida o índice do parâmetro (ex: 0=Kp, 1=Ki, 2=Kd, etc.)
					//EN = in.data[1];
				}else{
					snprintf(resposta, MAX_OUT,			// Erro se o índice for maior que o número de parâmetros disponíveis
							"\r\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,				// Impede a configuração se o sistema estiver em Manual ou Automático
						"\r\nESTADO ERRADO!");
				print(resposta);
			}
			break;										// Finaliza a execução do comando PID

		default:
			print("\r\nERRO INTERNO!");						//Se não foi selecionado nenhum dos comandos
			break;
		}
		break;

	// Tratamento dos Erros da Análise Léxica/Sintática
	case ERR:											//Erro genérico
		print("\r\nERRO!");
		break;
	case ERR_PAR:										//Erro de parâmetros
		print("\r\nERRO DE PARAMETROS!");
		break;
	case ERR_CMD:										//Erro de comando
		print("\r\nCOMANDO NAO RECONHECIDO!");
		break;
	case ERR_OVR:										//Erro de overflow
		print("\r\nOVERFLOW DE PARAMETROS!");
		break;
	default:											//Erro por defeito
		print("\r\nERRO INTERNO!");
		break;
	}

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void read(int l){								// Leitura da posição e da velocidade
	float PosRad = (inc_pos*(2.0*M_PI))/960.0;	// Conversão da posição para radianos
	float PosGra = (inc_pos*360.0)/960.0;		// Conversão da posição para RPM

	float pulse = (float)inc_vel/(__HAL_TIM_GET_AUTORELOAD(&htim6)/1000.0);
	float VelRad = (pulse*(2.0*M_PI))/960.0;			// Conversão da velocidade em rad/s
	float VelGra = (pulse*60.0)/960.0;					// Conversão da velocidade em rpm
	inc_vel=0;
	if(RT!=1){					// Leitura da posição
		snprintf(resposta, MAX_OUT,
					"\r\nPos: %0.3f rad | %0.3f deg %c %d voltas - [%d]\r\n", PosRad, PosGra, dir?'+':'-', vol, l);
		print(resposta);
	}
	if(RT!=0)				// Leitura da velocidade
		snprintf(resposta, MAX_OUT,
				"\r\nVel: %0.3f rad/s | %0.3f rpm %c %d voltas - [%d]\r\n", VelRad, VelGra, dir?'+':'-', vol, l);
		print(resposta);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int l = 0;												//Volta atual
int main_loop(void){
	char resposta[MAX_OUT] = {};						//Resposta ao utilizador
	if(data_ready == 1){ 								//Verifica se a flag de receção foi ativada
		upperCase(input);								//Se foi, garante que todos os comandos recebidos ficam apenas em maiúsculas (por causa do Case Sensitivity)
		// 1. parse(): Divide a string em tokens e valida a sintaxe (Análise Léxica/Sintática)
		// 2. execute(): Recebe o resultado do parse e atua no hardware (Semântica/Execução)
		execute(parse(input, delim));
		//print("\n[MANUAL]>");							// Imprime o prompt para indicar ao utilizador que o sistema está pronto para o próximo comando
		//Limpeza e Preparação para o próximo ciclo
		data_ready = 0;									//Reset da flag de receção
		memset(input, 0, MAX_CHAR); 					//Limpa o buffer de entrada para evitar resíduos de comandos anteriores (por segurança)

		// Reinício da escuta da UART
		if(start_scan(rx_buff) != HAL_OK){				// Tenta reativar o modo de receção
		 __HAL_UART_CLEAR_OREFLAG(&huart3);				//Limpa o erro de Overrun para desbloquear o periférico
		 start_scan(rx_buff);								// Segunda tentativa de arranque após limpeza do erro
		}
		return 1;
	}
	if(data_ready == 2){
		data_ready = 0;									//Reinicia a flag de receção
		snprintf(resposta,MAX_OUT, "%c", rx_buff[0]);
		print(resposta);
		if(start_scan(rx_buff) != HAL_OK){				// Tenta reativar o modo de receção
		 __HAL_UART_CLEAR_OREFLAG(&huart3);				//Limpa o erro de Overrun para desbloquear o periférico
		 start_scan(rx_buff);							// Segunda tentativa de arranque após limpeza do erro
		}
	}
	if (ov){											// Verifica se a flag de overflow está ativa
		ov = 0;											// Reset da flag (acknowledge)
	  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);			// Indicação visual que o código não "encravou" e o loop principal continua a correr
		switch(R){										//Casos de leitura
		case 0:											//Ler só posição
			break;
		case 1:											//Ler só velocidade
			read(l);
			break;
		case 2:											//Ler posição e velocidade
			if(l++<laps){
				read(l);
			}else{
				R=0;
				l=0;
			}
			break;
		default:
			l=0;
			R=0;
			break;
		}
	}
	if(lim){										//Se atingimos o limite de voltas
		lim=0;
		print("\r\nLimite de voltas atingido");
	}
	return 0;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void state_machine(){
	switch(CS){												//Switch Case para todos os casos
	case 0:													//Estado 0: Reset
		print("\r\n\r\nRESET- - - - - - - - - - -");		//Confirmação do estado
		//EN
		enable(0);											//Desativação dos pinos de enable
		//HW
		__HAL_TIM_SET_AUTORELOAD(&htim6, 10-1);				//Período de 10ms
		__HAL_TIM_SET_COUNTER(&htim6, 0);					//Reset do contador
		//RT
		RT=1;												//Leitura de velocidade
		//PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    //Canal de direção -
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); 	//Canal de direção +
		dir=1;												//Flag de direção
		//CS
		CS = 1;												//Por defeito, passamos automaticamente do estado 0 para o estado 1
		break;
	case 1:													//Estado 1: Configuração
		print("\r\n\r\nCONFIG- - - - - - - - - - -");		//Confirmação do estado
		print("\r\n[CONFIG]>");								//Prompt visual para o utilizador
		enable(0);											//Desativação dos pinos de enable
		while (CS==1){
			if(main_loop()){
			print("\r\n[CONFIG]>");							// Prompt visual para o utilizador
			}
		}
		break;
	case 2:													// Estado 2: Modo Manual
		print("\r\n\r\nMANUAL- - - - - - - - - - -");		// Confirmação do estado
		print("\r\n[MANUAL]>");								// Prompt visual para o utilizador
		while (CS==2){										// Permite o controlo direto e a leitura de sensores em malha aberta
			if(main_loop()){
			print("\r\n[MANUAL]>");							// Prompt visual para o utilizador
			}
		}
		break;
	case 3:													//Estado 3: Modo Automático
		print("\r\n\r\nAUTO- - - - - - - - - - -");			//Confirmação do estado
		print("\r\n[AUTO]>");								// Prompt visual para o utilizador
		enable(1);											//Ativação dos pinos de enable
		while (CS==3){ 										//Enquanto ficarmos neste estado
			if(main_loop()){
			print("\r\n[AUTO]>");							// Prompt visual para o utilizador
			}
			// Se o motor for desativado pelo comando enable (EN=0), regressa ao modo de configuração (1)
			if(!EN){
				CS=1;
			}
		}
		break;
	default:												//Por defeito, começa-se sempre no estado 0
		CS = 0;
		break;
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  start_scan(rx_buff);									//Início da receção pela usart3
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  //Bloco de processamento de comandos (consumidor)
	  state_machine();									//Chama a máquina de estados
	  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
volatile int comp = 0;							//Variável para se saber o valor atual do PWM
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){	//Callback de Evento de Receção UART: Executado automaticamente pelo hardware
char resposta[MAX_CHAR];									//Resposta ao utilizador
int arr = __HAL_TIM_GET_AUTORELOAD(&htim3);					//Valor do registo de Auto-Reload do timer3
    if (huart->Instance == USART3){							//Verifica se a interrupção veio da USART3 (ligada ao ST-Link/Terminal do PC)
        //input[Size] = '\0';								//Coloca o terminador nulo ('\0') exatamente na posição após o último caractere recebido
    	data_ready = 2;										//Não foi uma receção completa, por caracter
    	int inc = (5*(arr+1))/100;							//(Vcc*<signVal>) / 100
        if(rx_buff[0]=='\\'){								//Se o utilizador escreveu backslash
        	if(CS == 2){
				if(dir){
					comp = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
					comp -= inc;
					if(comp<0){
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
						comp=-comp;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, comp);
						dir=0;
					}else{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, comp);
					}
				}else{
					comp = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
					comp += inc;
					if(comp>arr){
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, arr);
						comp=arr;
					}else{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, comp);
					}
				}
				snprintf(resposta, MAX_OUT,					// Mostra o valor atual do PWM depois de backslash
								"\r\nPWM: %c%d > ", dir?'+':'-',((comp+1)*100/(arr+1)));
				print(resposta);
        	}else{
        		snprintf(resposta, MAX_OUT,				// Impede o controlo manual do PWM se o sistema estiver em Modo Automático (Estado 3)
        						"\r\nESTADO ERRADO!");
        		print(resposta);
        	}
        }else if(rx_buff[0]=='/'){ //Se o utilizador escreveu /
        	if(CS==2){
				if(dir){
					comp = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
					comp += inc;
					if(comp>arr){
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr);
						comp=arr;
					}else{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, comp);
					}
				}else{
					comp = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
					comp -= inc;
					if(comp<0){
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
						comp=-comp;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, comp);
						dir=1;
					}else{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, comp);
					}
				}
				snprintf(resposta, MAX_OUT,					// Mostra o valor atual do PWM depois de barra
								"\r\nPWM: %c%d > ", dir?'+':'-', ((comp+1)*100/(arr+1)));
				print(resposta);
        	}else{
        		snprintf(resposta, MAX_OUT,				// Impede o controlo manual do PWM se o sistema estiver em Modo Automático (Estado 3)
        						"\r\nESTADO ERRADO!");
        		print(resposta);
        	}
        }else{
        	int current_len = strlen(input); //Mostra o que temos até agora
        	if (rx_buff[0] == 0x08 || rx_buff[0] == 0x7F) { //Se escrevermos um delete
        	    if (current_len > 0) {
        	        input[current_len - 1] = '\0'; // Remove o último caractere
        	        print(" \b");
        	    }
        	}else{
        		if (current_len < (MAX_CHAR - 1)) {
        	     	 input[current_len] = rx_buff[0];
        	     	 input[current_len + 1] = '\0'; //Caracter delimitador nulo
        		}
        	}
            if (rx_buff[0] == '\r' || rx_buff[0] == '\n') {
                 data_ready = 1;				//A flag de receção foi ativada
            }
        }
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //Callback do Período do Timer: Executado quando o contador do Timer atinge o valor de Auto-Reload (ARR)
	if (htim == &htim6){ 				// Filtra para garantir que estamos a reagir apenas ao Timer 6
		ov = 1; 						// Sinaliza que o período de tempo decorreu (Overflow).
										//As ISR devem ser o mais curtas possível!
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ENC_A_Pin){ 				// Interrupção gerada pela transição de estado do Canal A do encoder

		// Lê o estado do Canal B para determinar o sentido de rotação
		if(HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_RESET){
			// Sentido Anti-horário
			inc_pos--;
			inc_vel--;
			//print("-");
		}else{
			// Sentido Horário
			inc_pos++;
			inc_vel++;
			//print("+");
		}
		// A caixa redutora gera 1920 pulsos por volta considerando os canais A e B
		// Se estivermos a contar apenas interrupções de um canal (descendentes ou ascendentes), 960 corresponde a uma volta

		if(abs(inc_pos)>=960){
			inc_pos=0;					// Reinicia o contador da posição

			if(inc_vel>0){
				vol++;					// Incrementa uma volta completa

			}else{
				vol--;					// Decrementa uma volta completa
			}
			//lim=1;
		}

		// Mecanismo de proteção: se exceder 10 voltas num sentido, o motor é desativado
		if(abs(vol) > VOL){
			enable(0);					//Desativa o motor
			lim = 1;
			vol = 0;					//Reinicia o nº de voltas
		}
	}

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
