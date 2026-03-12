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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Trabalho realizado por:
 * Nuno Costa a107069
 * Afonso Carvalho a107058
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
#define MEM 65536			//Limite dos caracteres hexadecimais (0 até 16^4)

#define TIM3_ARR  (64000-1)	// Valor de início do Auto-reload register
#define TIM3_CCR  (32000-1) // Valor de início do Capture-Compare Register

#define MAX_ADC 10			//Timeout de 10ms
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t memory[MEM]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; //Buffer que atua como memória virtual para os comandos MR e MW
const char delim[MAX_DELIM] = " ";										  //Declaração e inicialização dos caracteres delimitadores
char input[MAX_CHAR] = "";												  //Declaração e inicialização do buffer de entrada da uart
volatile uint8_t data_ready = 0;										  //Declaração e inicialização da flag de receção da uart
int pins[16];															  //Declaração e inicialização do vetor de pinos ativos pelos comandos
volatile uint8_t ov;													  //Variável para a flag de interrupção genérica (overflow)

int CS=0;
int last_CS=0;
int EN=0;

typedef enum{                           // Enumeração dos tipos de parâmetros esperados na análise sintática
    /*HEX = 0,                          // Parâmetro hexadecimal
    DEC,                                // Parâmetro hexadecimal, apresentado como decimal
    PORT,                               // Parâmetro de 'A' a 'Z'
    PIN,                                // Parâmetro hexadecimal, apresentado como string de pinos ativos, obtidos como binário
    VALUE,                              // Parâmetro hexadecimal, apresentado como string de valores binários
    DUTY,*/                             // Parâmetro hexadecimal, apresentado como decimal (para o duty-cycle)
    INT = 0,                            // Parâmetro para números inteiros (ex: os endereços de memória para MR/MW)
    DEC,                                // Parâmetro para números decimais (ex: o duty-cycle para o PWM)
    PIN,                                // Parâmetro hexadecimal, representando o mapa dos pinos (ex: o mapa de pinos para PI/PO/RD/WD)
    BIN,                                // Parâmetro hexadecimal, representando os valores lógicos (ex: os valores dos pinos para WD)
    CHAR,                               // Parâmetro caractere (ex: identificação da porta 'A', 'B', 'C', 'D', 'E', 'F' e 'G')
	DIG,
	FLOAT,
	SIGN,
}Type;

typedef enum{							//Enumeração dos tipos de erros
	ALL_OK = 0,							//Sem erros
	ERR,								//Erro genérico
	ERR_CMD,							//Erro de comando
	ERR_PAR,							//Erro de parâmetro
	ERR_OVR,								//Erro de overflow
	//ERR_STATE,
}Error;

typedef enum{							//Enumeração dos tipos de comandos a usar
	CMD_MR = 0,							//Comando Memory Read
	CMD_MW,								//Comando Memory Write
	CMD_PI,								//Comando Make Pins Input
	CMD_PO,								//Comando Make Pins Output
	CMD_RD,								//Comando Read Digital Input
	CMD_WD,								//Comando Write Digital Output
	CMD_PWMS,							//Comando Pulse Width Modulation
	CMD_RA,								//Comando Analog Read
	CMD_HELP,							//Comando Help
	CMD_CS,
	CMD_EN,
	CMD_HW,
	CMD_RT,
	CMD_R,
	CMD_PWM,
	CMD_RESPOS,
	CMD_PID,
}Cmd;

typedef struct {                        // Estrutura gerada pelo analisador léxico/sintático (parser)
    int data[10];                       // Vetor de tokens convertidos em inteiros/dados numéricos
    Error state;                        // Regista se ocorreu algum erro durante o parsing da string
}Tokens;

typedef struct {                        // Estrutura do mapeamento abstrato de uma porta GPIO
    GPIO_TypeDef *channel;              // Apontador para a estrutura base da porta (ex: GPIOA, GPIOB)
    uint16_t pin_mask;                  // Máscara com os pinos físicos que estão efetivamente disponíveis na placa
}Port;

typedef struct {                        // Estrutura da tabela de pesquisa de portas
    char id;                            // Identificador em caractere ('A', 'B', etc.)
    GPIO_TypeDef *channel;              // Apontador para o respetivo periférico GPIO
    uint16_t mask;                      // Máscara de pinos permitidos/seguros para manipulação
} PortEntry;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Tabela constante de pesquisa. Mapeia a letra da porta introduzida pelo utilizador para o endereço do hardware.
// Impede que o utilizador tente configurar pinos inexistentes ou reservados pela nossa placa (Nucleo-H755)
static const PortEntry port_table[] = {
    {'A', GPIOA, (1<<0 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<15)},
    {'B', GPIOB, (1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<15)},
    {'C', GPIOC, (1<<0 | 1<<2 | 1<<3 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12)},
    {'D', GPIOD, (1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15)},
    {'E', GPIOE, (1<<0 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15)},
    {'F', GPIOF, (1<<7 | 1<<8 | 1<<9)},
    {'G', GPIOG, (1<<6 | 1<<12 | 1<<14)}
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Port select_port(char port_char) {                              									// Função de pesquisa da porta pelo endereço da mesma (<portAddr>)
    int i=0;																						//Declaração e inicialização do iterador
    for (i = 0; i < (sizeof(port_table) / sizeof(PortEntry)); i++) { 								// Percorre a tabela das portas
        if (port_table[i].id == port_char) {                   										// Se o caractere recebido coincidir com o endereço da porta em questão
            return (Port){ .channel = port_table[i].channel, .pin_mask = port_table[i].mask }; 		// Retorna a configuração
        }
    }
    return (Port){ .channel = NULL, .pin_mask = 0 };            									// Retorna NULL se a porta não existir (útil para validar erros)
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Error pins_restricted(Port port, int pins){ //Retorna ERR se houver uma tentativa de acesso a bits fora da máscara permitida e ALL_OK caso contrário
	return ((pins & ~port.pin_mask) != 0 ? ERR : ALL_OK);
}
void start_GPIO_CLK(char p){ 						//Função para ativar os clocks das respetivas portas do GPIO
	switch(p){
		case 'A':
			__HAL_RCC_GPIOA_CLK_ENABLE();			//Ativar o clock da porta A
			break;
		case 'B':
			__HAL_RCC_GPIOB_CLK_ENABLE();			//Ativar o clock da porta B
			break;
		case 'C':
			__HAL_RCC_GPIOC_CLK_ENABLE();			//Ativar o clock da porta C
			break;
		case 'D':
			__HAL_RCC_GPIOD_CLK_ENABLE();			//Ativar o clock da porta D
			break;
		case 'E':
			__HAL_RCC_GPIOE_CLK_ENABLE();			//Ativar o clock da porta E
			break;
		case 'F':
			__HAL_RCC_GPIOF_CLK_ENABLE();			//Ativar o clock da porta F
			break;
		case 'G':
			__HAL_RCC_GPIOG_CLK_ENABLE();			//Ativar o clock da porta G
			break;
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : -1;
}*/
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

Error validate(char par[], int digits, int type, int *out){ 		//Função para validar os parâmetros implementados pelo utilizador
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
		num = (uint32_t)strtoul(par, NULL, 16);						//Conversão da string do parâmetro para um unsigned long int
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
		    if(par[1] != '\0') return ERR;                  // Garante que é apenas um caractere
		    if(par[0] < 'A' || par[0] > 'Z') return ERR;    // Valida se está no intervalo ASCII de letras maiúsculas
		    *out = par[0];                                  //Novo valor do endereço da string de saída
		    break;
		case DIG:
			if(num < 10){
				*out = num;
				break;
			}
			return ERR;
			break;
		case FLOAT:
			if((par[1]!='.') && (par[1]!=',')) return ERR;
			for(i=0; i<digits; i+=2){									//Ciclo para verificação dos dígitos
				if(par[i]=='\0') break;									//Se a string não utilizar todos os caracteres disponíveis, acaba o ciclo
				if(!isxdigit(par[i])) return ERR;						//Se o utilizador escrever algo que não seja um dígito hexadecimal, retorna erro
				if((uint32_t)strtoul(par[i], NULL, 16)>9) return ERR;						//Se o utiliz, retorna erro
			}
			int uniF = par[0]-'0';
			int decF = par[2]-'0';
			num = uniF*10 + decF;
			*out = num;
			break;
		case SIGN:
			switch(par[0]){
			case ' ':
				*out = 1;
				break;
			case '+':
				*out = 1;
				break;
			case '-':
				*out = -1;
				break;
			default:
				return ERR;
				break;
			}
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

	}else if(strcmp(in[0], "MR") == 0){											////Verificação se é o comando de leitura de memória
		out.data[0] = CMD_MR;													//Ativa o comando MEMORY READ
		if(count==3){															//Verificação do número de parâmetros
			char *addrStr = in[1];												//Declaração e inicialização da string do endereço da memória
			char *lenStr = in[2];												//Declaração e inicialização da string do nº de bytes
			if((validate(addrStr, 4, INT, &(out.data[1])))						//Validação do endereço, com 4 dígitos, sendo inteira
					||(validate(lenStr, 2, INT, &(out.data[2])))){				//Validação do comprimento, com 2 dígitos, sendo inteira
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "MW") == 0){											////Verificação se é o comando de escrita na memória
		out.data[0] = CMD_MW;													//Ativa o comando MEMORY WRITE
		if(count==4){															//Verificação do número de parâmetros
			char *addrStr = in[1];												//Declaração e inicialização da string do endereço de memória
			char *lenStr = in[2];												//Declaração e inicialização da string do nº de bytes
			char *byteStr = in[3];												//Declaração e inicialização da string do valor do byte
			if((validate(addrStr, 4, INT, &(out.data[1])))						//Validação do endereço, com 4 dígitos, sendo inteiro
					||(validate(lenStr, 2, INT, &(out.data[2])))				//Validação do comprimento, com 2 dígitos, sendo inteiro
						||(validate(byteStr, 2, INT, &(out.data[3])))){     	//Validação do valor, com 2 dígitos, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "PI") == 0){											////Verificação se é o comando de configuração dos pinos de entrada
		out.data[0] = CMD_PI;													//Ativa o comando MAKE PINS INPUT
		if(count==3){															//Verificação do número de parâmetros
			char *portStr = in[1];												//Declaração e inicialização da string do endereço da porta
			char *pinStr = in[2];												//Declaração e inicialização da string do mapa de pinos
			if((validate(portStr, 1, CHAR, &(out.data[1])))						//Validação da porta, com 1 dígito, sendo um caractere (uma letra)
					||(validate(pinStr, 4, PIN, &(out.data[2])))){ 				//Validação dos pinos, com 4 dígitos, sendo a lista dos pinos
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "PO") == 0){											////Verificação se é o comando de configuração dos pinos de saída
		out.data[0] = CMD_PO;													//Ativa o comando MAKE PINS OUTPUT
		if(count==3){															//Verificação do número de parâmetros
			char *portStr = in[1];												//Declaração e inicialização da string do endereço da porta
			char *pinStr = in[2];												//Declaração e inicialização da string do mapa de pinos
			if((validate(portStr, 1, CHAR, &(out.data[1])))						//Validação da porta, com 1 dígito, sendo um caractere (uma letra)
					||(validate(pinStr, 4, PIN, &(out.data[2])))){				//Validação dos pinos, com 4 dígitos, sendo a lista de pinos
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "RD") == 0){											////Verificação se é o comando de leitura digital dos pinos
		out.data[0] = CMD_RD;													//Ativa o comando READ DIGITAL INPUT
		if(count==3){															//Verificação do número de parâmetros
			char *portStr = in[1];												//Declaração e inicialização da string do endereço da porta
			char *pinStr = in[2];												//Declaração e inicialização da string do mapa de pinos
			if((validate(portStr, 1, CHAR, &(out.data[1])))						//Validação da porta, com 1 dígito, sendo um caractere (uma letra)
					||(validate(pinStr, 4, PIN, &(out.data[2])))){  			//Validação dos pinos, com 4 dígitos, sendo a lista de pinos
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "WD") == 0){											////Verificação se é o comando de escrita digital dos pinos
		out.data[0] = CMD_WD;													//Ativa o comando WRITE DIGITAL OUTPUT
		if(count==4){															//Verificação do número de parâmetros
			char *portStr = in[1];												//Declaração e inicialização da string do endereço da porta
			char *pinStr = in[2];												//Declaração e inicialização da string do mapa de pinos
			char *valStr = in[3];												//Declaração e inicialização da string do valor do pino
			if((validate(portStr, 1, CHAR, &(out.data[1])))						//Validação da porta, com 1 dígito, sendo um caractere (uma letra)
					||(validate(pinStr, 4, PIN, &(out.data[2])))				//Validação dos pinos, com 4 dígitos, sendo alista de pinos
						||(validate(valStr, 4, BIN, &(out.data[3])))){			//Validação dos valores, com 4 dígitos, sendo os valores lógicos dos pinos
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}

			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "PWMS") == 0){										////Verificação se é o comando de PWM
		out.data[0] = CMD_PWMS;													//Ativa o comando PULSE WIDTH MODULATION
		if(count==2){															//Verificação do número de parâmetros
			char *dutyStr = in[1];												//Declaração e inicialização da string do duty-cycle
			if(validate(dutyStr, 2,INT, &(out.data[1]))){						//Validação do duty-cycle, com 2 dígitos,em uma variável inteira
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída

	}else if(strcmp(in[0], "RA") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_RA;													//Ativa o comando ANALOG READ
		if(count==2){															//Verificação do número de parâmetros
			char *addrStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			if(validate(addrStr, 1, INT, &(out.data[1]))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;																//Retorna o erro de parâmetros na string de saída
	//--------------------------------
	}else if (strcmp(in[0], "CS") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_CS;													//Ativa o comando ANALOG READ
		if(count==2){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			if(validate(digitStr, 1, DIG, &(out.data[1]))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "EN") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_EN;													//Ativa o comando ANALOG READ
		if(count==2){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			if(validate(digitStr, 1, DIG, &(out.data[1]))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "HW") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_HW;													//Ativa o comando ANALOG READ
		if(count==2){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			if(validate(digitStr, 4, INT, &(out.data[1]))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "RT") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_RT;													//Ativa o comando ANALOG READ
		if(count==2){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			if(validate(digitStr, 1, DIG, &(out.data[1]))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "R") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_R;													//Ativa o comando ANALOG READ
		if(count==3){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			char *unitStr = in[2];
			if((validate(digitStr, 1, DIG, &(out.data[1])))
					||(validate(unitStr, 4, INT, &(out.data[2])))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "PWM") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_PWM;													//Ativa o comando ANALOG READ
		if(count==2){	//Verificação do número de parâmetros
			char signStr[2];
			char digitStr[4];
			if(in[1][0] < '0' || in[1][0] > '9'){
				signStr[0] = in[1][0];
				digitStr[0] = in[1][1];
				digitStr[1] = in[1][2];
				digitStr[2] = in[1][3];
			}else{
				strcpy(signStr," ");
				strcpy(digitStr,in[1]);
			}
														//Declaração e inicialização da string do endereço do canal 3 do ADC
			if((validate(signStr, 1, SIGN, &(out.data[1])))
					||(validate(digitStr, 3, INT, &(out.data[2])))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "RESPOS") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_RESPOS;													//Ativa o comando ANALOG READ
		if(count==1){															//Verificação do número de parâmetros
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}else if (strcmp(in[0], "PID") == 0){											////Verificação se é o comando de leitura analógica do ADC (Analog to Digital Converter)
		out.data[0] = CMD_PID;													//Ativa o comando ANALOG READ
		if(count==3){															//Verificação do número de parâmetros
			char *digitStr = in[1];												//Declaração e inicialização da string do endereço do canal 3 do ADC
			char *floatStr = in[2];
			if((validate(digitStr, 1, DIG, &(out.data[1])))
					||(validate(floatStr, 3, FLOAT, &(out.data[2])))){						//Validação do endereço, com 1 dígito, sendo inteiro
				out.state = ERR_PAR;											//Se a validação der errado, estamos com erro de parâmetros
				return out;														//Retorna o erro de parâmetros na string de saída
			}
			out.state = ALL_OK;													//Confirmação da validação (tudo bem)
			return out;															//Retorna o comando escolhido na string de saída
		}
		out.state = ERR_PAR;													//Se a validação der errado, estamos com erro de parâmetros
		return out;
	}
	out.state = ERR_CMD;														// Se não coincidir com nenhum comando conhecido, dá erro de comando
	return out;																	//Retorna o erro de comando na string de saída
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void list(int num, char out[64], Type t, int pin){  //Função para ajudar na lista de pinos
	int pins[16]={};								// Vetor temporário para guardar os índices dos pinos ativos
	int data[16]={};								// Vetor para guardar os dados finais a apresentar
	char temp[64]={};								// String auxiliar temporária para usar com o sprintf
	int n=0;										//Reset do índice do vetor
	int i=0;										//Declaração e inicialização do iterador
	pins[0] = '\0';									//Reset do array de pinos

	// Descodificação da máscara de pinos (num)
	for (i=0; i<16; i++){							//Ciclo a percorrer todos os pinos da porta GPIO do STM32 (0 a 15)
		if(num>>i & 0x01){							// Desloca o bit 'i' para a posição 0 e isola-o com a máscara AND 0x01
			pins[n++]=i;							// Se o bit for 1, o pino 'i' foi selecionado. Guarda o índice e avança 'n'
		}
	}

	//Preparação dos dados a mostrar consoante o tipo pedido
	if(t == BIN){									// Se for o comando WD (Write Digital), queremos os valores a escrever
		int bin[16] = {0};							// Vetor intermédio para mapear os 16 bits de valor 1 ou 0
		for (i=0; i<16; i++){						//Ciclo a percorrer os 16 bits do parâmetro 'pin' (<pinValues>)
			if(pin>>i & 0x01){						// Extrai o bit individual do valor lógico
				bin[i]=1;							// Regista que o pino 'i' deve receber o estado lógico ALTO (1)
			}
		}
		for (i=0; i<n; i++){						//Ciclo de filtragem, usando apenas os pinos que foram efetivamente selecionados
			data[i]=bin[pins[i]];					// Guarda no vetor de saída o valor lógico (0 ou 1) correspondente àquele pino
		}
	}else{											// Para os comandos PI, PO ou RD, queremos apenas os NÚMEROS dos pinos
		for (i=0; i<n; i++){						//Ciclo a percorrer o vetor de pinos selecionados
			data[i]=pins[i];						// O dado a mostrar é o índice físico do pino
		}
	}
	out[0]='\0';									//Reset da string de saída

	if (num>0){										// Se a máscara não estiver a zeros (pelo menos um pino selecionado)
		sprintf(temp, "%d", data[0]); 				//Formata o primeiro elemento da lista (pino ou valor). Converte o pino para string
		strcat(out, temp);          				// Adiciona este primeiro elemento à string de saída

		for(i=1;i<n;i++){							//Ciclo que percorre os restantes elementos encontrados
			if(i!=(n-1)){							// Se não for o último elemento da lista
				sprintf(temp, ", %d", data[i]); 	// Prepara o separador com vírgula (ex: "1, 2")
				strcat(out, temp);         			// Concatena na string principal
			}else{									// Se for o último elemento da lista
				sprintf(temp, " e %d", data[i]); 	// Coloca a conjunção "e" para fechar a lista (ex: "1, 2 e 5")
				strcat(out, temp);          		// Concatena na string principal
			}
		}
	}else{											// Se nenhum pino foi selecionado (num == 0)
		sprintf(temp, "n/a "); 						// Informa que o resultado não é Aplicável (n/a)
		strcat(out, temp);         					// Concatena na string principal
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Error check_state(int states[4]){
	int i=0;
	for(i=0;i<4;i++){
		if(CS==states[i]) return ALL_OK;
	}
	return ERR;
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
ADC_ChannelConfTypeDef ADC_CH_Cfg = {0}; //Inicialização da configuração do canal do ADC
//Lista de canais ADC
uint32_t ADC_Channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15, ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_19};
void execute(Tokens in){ //Função execute
	// Buffers locais para montagem das mensagens de resposta ao utilizador
	char resposta[MAX_OUT] = {};						//Resposta ao utilizador
	char active[64]={};									// Guarda a lista de pinos formatada (ex: "0, 1 e 2")
	char value[64]={};									// Guarda a lista de valores lógicos formatada
	char out[MAX_OUT]={};								// Buffer auxiliar para concatenação de strings longas (diferente do out referido no código anterior)
	char temp[64]={};									// String temporária para conversões rápidas
	int i=0;											//Declaração e inicialização do iterador
	Port port;											// Estrutura para manipular a porta selecionado
	GPIO_InitTypeDef GPIO_InitStruct = {0};				// Estrutura HAL para configuração dos pinos
	switch(in.state){									// Primeiro nível de decisão: Verifica se o comando é válido antes de tentar executar
	case ALL_OK:
		switch(in.data[0]){								// Segundo nível de decisão: Identifica qual o comando específico que foi solicitado

		case CMD_HELP:									// Comando '?' - Lista de ajuda
			print("Comandos disponiveis:\n");
			print("?                                                                              - Fornece uma lista dos comandos validos.\n");
			print("MR <addr> <length>                                         - Ler <length> bytes a partir de endereco de memoria <addr>h\n");
			print("MW <addr> <length> <byte>                          - Escrever <length> bytes a partir de endereco de memoria <addr>h com o valor <byte>h\n");
			print("PI <portAddr> <pinsMap>                              - Programar os pinos <pinsMap> da porta <portAddr> como entrada\n");
			print("PO <portAddr> <pinsMap>                            - Programar os pinos <pinsMap> da porta <portAddr> como saida\n");
			print("RD <portAddr> <pinsMap>                            - Ler os pinos <pinsMap> da porta <portAddr>\n");
			print("WD <portAddr> <pinsMap> <pinValues> - Escrever nos bits <pinsMap> da porta <portAddr>, os valores <pinValues>\n");
			print("PWM <dutyCycle>                                            - Duty-cycle de <dutyCycle>%\n");
			print("RA <addr>                                                           - Ler o canal <addr> de um ADC pre-determinado");
			break;
		case CMD_MR:									//Memory Read
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Ler %d bytes a partir de endereco de memoria %04Xh ", in.data[2], in.data[1]);
			print(resposta);							//Escrita da resposta
			sprintf(temp, "\nRead: ");
						strcat(out, temp);
			for(i=0; i<in.data[2]; i++){				// Ciclo para ler a memória virtual (vetor 'memory')
														// Proteção contra leitura fora de limites (Memory Wrap-around)
				sprintf(temp, "%X ", ((in.data[1]+i >= MEM )? memory[i] : memory[in.data[1]+i]));
				strcat(out, temp);						// Concatena na string principal
			}
			print(out);
			break;
		case CMD_MW:									//Memory Write
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Escrever %d bytes a partir de endereco de memoria %04Xh com o valor %02Xh ", in.data[2], in.data[1], in.data[3]);
			print(resposta);							//Escrita da resposta
			sprintf(temp, "\nWritten: ");
						strcat(out, temp);				// Concatena na string principal
			// Ciclo para escrever na memória virtual (vetor 'memory')
			for(i=0; i<in.data[2]; i++){
				// Escrita na memória virtual com verificação dos limites
				if((in.data[1]+i)>=MEM){
					memory[i]=in.data[3];
				}else{
					memory[in.data[1]+i]=in.data[3];
				}
				//memory[in.data[1]+i]=in.data[3];
				sprintf(temp, "%X ", ((in.data[1]+i >= MEM )? memory[i] : memory[in.data[1]+i])); 				//Converte o pino para string
				strcat(out, temp);						// Concatena na string principal
				}
			print(out);
			break;

		case CMD_PI:									//Make Pins Input
			list(in.data[2], active, PIN, 0);			// Converte a máscara hexadecimal para uma string legível
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Programar os pinos %s da porta %c como entrada", active, in.data[1]);
			print(resposta);							//Escrita da resposta

			port = select_port(in.data[1]);				// Obtém o endereço real do hardware (ex: GPIOA)
			if(port.channel == NULL){					//Se não conseguir, diz que a porta está fora do alcance
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPORTA FORA DE ALCANCE");
				print(resposta);			//Escrita da resposta
				return;
			}
			// Valida se os pinos solicitados não são restritos
			if(pins_restricted(port,in.data[2])){
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPINOS RESTRITOS!");
				print(resposta);			//Escrita da resposta
				return;
			}
			start_GPIO_CLK(in.data[1]);
			// Configuração física via HAL
			GPIO_InitStruct.Pin = in.data[2];			//Máscara de bits
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;		//Configurar os pinos para entrada
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;		//Garante o nível lógico 0 se nada estiver ligado
			HAL_GPIO_Init(port.channel, &GPIO_InitStruct); // Aplica a configuração do hardware: escreve nos registos do periférico (port.channel) as definições do modo e dos pinos guardados na estrutura GPIO_InitStruct
			snprintf(resposta, MAX_OUT,						//Apresentação da resposta ao comando
					"\nPinos definidos como entrada.");
			print(resposta);								//Escrita da resposta
			break;
		case CMD_PO:									//Make Pins Output
			list(in.data[2], active, PIN, 0);			// Converte a máscara hexadecimal para uma string legível
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Programar os pinos %s da porta %c como saida", active, in.data[1]);
			print(resposta);							//Escrita da resposta
			port = select_port(in.data[1]);				// Obtém o endereço real do hardware (ex: GPIOA)
			if(port.channel == NULL){					//Se não conseguir, diz que a porta está fora do alcance
				snprintf(resposta, MAX_OUT,				//Apresentação da resposta ao comando
						"\nPORTA FORA DE ALCANCE");
				print(resposta);						//Escrita da resposta
				return;
			}
			// Valida se os pinos solicitados não são restritos (ex: pinos do depurador ou oscilador)
				if(pins_restricted(port,in.data[2])){
					snprintf(resposta, MAX_OUT,			//Apresentação da resposta ao comando
							"\nPINOS RESTRITOS!");
					print(resposta);					//Escrita da resposta
					return;
			}
			start_GPIO_CLK(in.data[1]);
			// Configuração física via HAL
			GPIO_InitStruct.Pin = in.data[2];			//Máscara de bits (ex: 0x0001 para o pino 0)
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Configurar os pinos para saída
			HAL_GPIO_Init(port.channel, &GPIO_InitStruct);// Aplica a configuração do hardware: escreve nos registos do periférico (port.channel) as definições do modo e dos pinos guardados na estrutura GPIO_InitStruct
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
				"\nPinos definidos como saida.");
			print(resposta);							//Escrita da resposta
			break;
		case CMD_RD:									//Read Digital Input
			list(in.data[2], active, PIN, 0);			// Converte a máscara hexadecimal para uma string legível
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Ler os pinos %s da porta %c\n", active, in.data[1]);
			print(resposta);							//Escrita da resposta
			port = select_port(in.data[1]);				// Obtém o endereço real do hardware (ex: GPIOA)
			if(port.channel == NULL){					//Se não conseguir, diz que a porta está fora do alcance
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPORTA FORA DE ALCANCE");
				print(resposta);			//Escrita da resposta
				return;
			}
			// Valida se os pinos solicitados não são restritos
			if(pins_restricted(port,in.data[2])){
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPINOS RESTRITOS!");
				print(resposta);			//Escrita da resposta
				return;
			}
			snprintf(resposta, 1, "\0");// Limpa o buffer para a nova resposta
			for (i=0; i<16; i++){		//Ciclo a percorrer todos os pinos da porta
				if(in.data[2]>>i & 0x01){	// Se o bit 'i' estiver na máscara solicitada
					// Efetua a leitura real do pino físico
					int bit = HAL_GPIO_ReadPin(port.channel, (1 << i));
					sprintf(temp, "%d ", bit); //Converte o pino para string
					strcat(resposta, temp);	// Concatena na string principal
				}
			}
			print(resposta);								//Escrita da resposta
			snprintf(resposta, MAX_OUT,						//Apresentação da resposta ao comando
							"\nLer os pinos");
			print(resposta);							//Escrita da resposta
			break;
		case CMD_WD:									// Write Digital Output
			list(in.data[2], active, PIN, 0);			// Converte a máscara hexadecimal para uma string legível
			list(in.data[2], value, BIN, in.data[3]);	// Traduz os valores hexadecimais em binário (0's e 1's)
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Escrever nos bits %s da porta %c, os valores %s ", active, in.data[1], value);
			print(resposta);							//Escrita da resposta
			port= select_port(in.data[1]);				// Obtém o endereço real do hardware (ex: GPIOA)
			if(port.channel == NULL){		//Se não conseguir, diz que a porta está fora do alcance
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPORTA FORA DE ALCANCE");
				print(resposta);			//Escrita da resposta
				return;
			}
			// Valida se os pinos solicitados não são restritos
			if(pins_restricted(port,in.data[2]))/*||((port.channel->MODER >> (in.data[2] * 2)) & 0x03)==0x01)*/{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nPINOS RESTRITOS!");
				print(resposta);			//Escrita da resposta
				return;
			}
			for (i=0; i<16; i++){			//Ciclo a percorrer todos os pinos da porta
				if(in.data[2]>>i & 0x01){	// Para cada pino selecionado
					GPIO_PinState estado = (in.data[3] >> i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
					HAL_GPIO_WritePin(port.channel, (1 << i), estado);
					}
			}
			snprintf(resposta, MAX_OUT,		//Apresentação da resposta ao comando
					"\nEscrever os pinos");
			print(resposta);				//Escrita da resposta
			break;
		case CMD_PWMS:									//Pulse Width Modulation
			uint32_t num = /* 255 - */in.data[1];		//Valor de entrada
			num *= 100;									//Multiplicação para percentagem
			uint32_t uni = num / 255;					//Obtenção do valor inteiro (quociente)
			uint32_t dec = num % 255;					//Obtenção do valor decimal restante (resto)
			dec*=100;									//Multiplicação para percentagem da parte decimal
			dec/=255;									//Divisão em 255 da parte decimal

			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Duty-cycle de %d.%d%%", uni, dec);
			print(resposta);							//Escrita da resposta
			num /= 100;									//Valor de entrada a dividir por 100
						//uint16_t current_CCR = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);

			// Atualiza o registo de comparação do Timer (CCR) para mudar a largura do pulso
			// Próximo valor = (Proporção com 255) * Valor do Auto-Reload do Timer
			uint16_t next_CCR = (((num * (__HAL_TIM_GET_AUTORELOAD(&htim3))) / 255) > TIM3_ARR)  ? 0 : ((num * (__HAL_TIM_GET_AUTORELOAD(&htim3))) / 255);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, next_CCR);
			//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			break;

		case CMD_RA:									// Read Analog (ADC)
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Ler o canal %d de um ADC pre-determinado", in.data[1]);
			print(resposta);								//Escrita da resposta
			ADC_CH_Cfg.Rank =  ADC_REGULAR_RANK_1;			//ADC escolhido (ADC 1)
			ADC_CH_Cfg.Channel = ADC_Channels[in.data[1]];  //Escolher o canal a ser lido no ADC
			HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg);		//Confirmação das definições anteriores
			HAL_ADC_Start(&hadc1); 						// Ativa o periférico ADC
			if (HAL_ADC_PollForConversion(&hadc1, MAX_ADC) == HAL_OK) { // Aguarda a conversão (Polling) com o timeout definido (10ms)
				uint32_t val = HAL_ADC_GetValue(&hadc1);				// Valor de 12 bits (0-4095): (2^12) - 1 = 4096 - 1 = 4095
			    uint32_t v_total = (val * 330) / 4095;					// Converte para tensão: V = (Valor * 3.3V) / 4095. Multiplicado por 100 para evitar floats
			    int uni = v_total / 100;								//Obtenção do valor da tensão inteiro (quociente)
			    int dec = v_total % 100;								//Obtenção do valor decimal da tensão restante (resto)
		        snprintf(resposta, MAX_OUT, "\nADC: %d.%02dV ", uni, dec);	//Mensagem do read analog
		    } else {
		        snprintf(resposta, MAX_OUT, "\nErro: Timeout ADC ");	//Erro de timeout
		    }
		    print(resposta);				//Escrita da resposta
		    HAL_ADC_Stop(&hadc1);			//Desliga o ADC
			break;
		//-------------------------------
		case CMD_CS:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o estado como %d", in.data[1]);
			print(resposta);
			if(in.data[1]<=3){
				CS = in.data[1];
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nERRO DE VALOR!");
				print(resposta);
			}
			break;
		case CMD_EN:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o enable como %d", in.data[1]);
			print(resposta);
			int en[]={2,3,-1,-1};
			if(!check_state(en)){
				if(in.data[1]<=1){
					EN = in.data[1];
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_HW:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o periodo de amostragem como %d", in.data[1]);
			print(resposta);
			int hw[]={1,-1,-1,-1};
			if(!check_state(hw)){
				if(in.data[1]<=1000){
					//periodo = in.data[1]//(ms)
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_RT:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o modo de leitura como %d", in.data[1]);
			print(resposta);
			int rt[]={1,-1,-1,-1};
			if(!check_state(rt)){
				if(in.data[1]<=2){
					//
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_R:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define a leitura como %d e %d", in.data[1], in.data[2]);
			print(resposta);
			int r[]={2,-1,-1,-1};
			if(!check_state(r)){
				if(in.data[1]<=2){
					//
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_PWM:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o PWM como %d%% com o sinal %d", in.data[2],in.data[1]);
			print(resposta);
			int pwm[]={1,2,-1,-1};
			if(!check_state(pwm)){
				if(abs(in.data[2])<=100){
					//EN = in.data[1];
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_RESPOS:									//
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Reinicia a posicao");
			print(resposta);
			int respos[]={1,2,-1,-1};
			if(!check_state(respos)){
				//
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		case CMD_PID:									//
			uint32_t numP = in.data[2];		//Valor de entrada
			uint32_t uniP = numP / 10;					//Obtenção do valor inteiro (quociente)
			uint32_t decP = numP % 10;					//Obtenção do valor decimal restante (resto)
			snprintf(resposta, MAX_OUT,					//Apresentação da resposta ao comando
					"Define o PID como %d e posicaoo %d.%d", in.data[1], uniP, decP);
			print(resposta);
			int pid[]={1,-1,-1,-1};
			if(!check_state(pid)){
				if(abs(in.data[1])<=5){
					//EN = in.data[1];
				}else{
					snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
							"\nERRO DE VALOR!");
					print(resposta);
				}
			}else{
				snprintf(resposta, MAX_OUT,	//Apresentação da resposta ao comando
						"\nESTADO ERRADO!");
				print(resposta);
			}
			break;
		default:
			print("ERRO INTERNO!");						//Se não foi selecionado nenhum dos comandos
			break;
		}
		break;

	// Tratamento dos Erros da Análise Léxica/Sintática
	case ERR:											//Erro genérico
		print("ERRO!");
		break;
	case ERR_PAR:										//Erro de parâmetros
		print("ERRO DE PARAMETROS!");
		break;
	case ERR_CMD:										//Erro de comando
		print("COMANDO NAO RECONHECIDO!");
		break;
	case ERR_OVR:										//Erro de overflow
		print("OVERFLOW DE PARAMETROS!");
		break;
//	case ERR_STATE:										//Erro de estados
//		print("ERRO DE ESTADOS!");
//		break;
	default:											//Erro por defeito
		print("ERRO INTERNO!");
		break;
	}

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void main_loop(void){
	if(data_ready == 1){ 								//Verifica se a flag de receção foi ativada
		upperCase(input);								//Se foi, garante que todos os comandos recebidos ficam apenas em maiúsculas (por causa do Case Sensitivity)
		// 1. parse(): Divide a string em tokens e valida a sintaxe (Análise Léxica/Sintática)
		// 2. execute(): Recebe o resultado do parse e atua no hardware (Semântica/Execução)
		execute(parse(input, delim));
		print("\n[MANUAL]>");									// Imprime o prompt para indicar ao utilizador que o sistema está pronto para o próximo comando
		//Limpeza e Preparação para o próximo ciclo
		data_ready = 0;								//Reset da flag de receção
		memset(input, 0, MAX_CHAR); 					//Limpa o buffer de entrada para evitar resíduos de comandos anteriores (por segurança)
		// Reinício da escuta da UART
		if(start_scan(input) != HAL_OK){				// Tenta reativar o modo de receção
		 __HAL_UART_CLEAR_OREFLAG(&huart3);		//Limpa o erro de Overrun para desbloquear o periférico
		 start_scan(input);						// Segunda tentativa de arranque após limpeza do erro
		}
	}
	if (ov){											// Verifica se a flag de overflow está ativa
		ov = 0;									// Reset da flag (acknowledge)
	  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	// Indicação visual que o código não "encravou" e o loop principal continua a correr
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void state_machine(){
	switch(CS){
	case 0:				//Reset
		print("\nRESET---------------------");
		/*
		Desativa os pinos de enable, de definição de sentido de rotação e de sinal de PWM;
		Modo de limpeza das configurações iniciais do utilizador (período de amostragem – 10 ms; tipo de leitura - leitura de posição; reset de posição, PWM - duty cycle 0%, valores dos parâmetros PID – todos a zero);
		Deve ser desativado qualquer periférico em atuação;
		Transição de estado: o estado 0 deve ser transitório, passando para o estado 1 (modo de configuração) após o reset das variáveis ser concluído.
		*/
		EN = 0;


		CS = 1;
		break;
	case 1:				//Config
		print("\nCONFIG---------------------");
		print("\n[CONFIG]>");
		/*
		Este modo serve para o utilizador configurar diferentes variáveis necessárias para o controlo:
			Configuração do período de amostragem (utilizador);
			Configuração do tipo de leitura a ser realizada (utilizador);
			Configuração do valor inicial do duty-cycle do sinal de PWM (utilizador);
			Configuração da posição zero do disco - contador de posição inicializado (utilizador);
			Configuração da posição desejada do disco e dos parâmetros do controlador em malha fechada (utilizador);
			Configuração dos periféricos necessários (microcontrolador).
		Subentende-se que sempre que este estado é iniciado, deve-se limpar os somatórios de erros existentes no controlador PID;
		Subentende-se que ao entrar neste modo de funcionamento, a variável de Enable do motor (EN) deve tomar o valor de 0, e o microcontrolador deve inicializar os restantes periféricos (PWM, GPIO, …);
		Transição de estado:
			O estado 1 (Modo de configuração) pode ser selecionado a partir de qualquer um dos outros estados;
			A partir do estado 1 (Modo de configuração) é possível selecionar qualquer outro estado da máquina de estados.
		*/
//		if(last_CS != 1){
//			EN = 0;
//		}
		EN = 0;
		while (CS==1){
			main_loop();
		}
		break;
	case 2:				//Manual
		print("\nMANUAL---------------------");
		print("\n[MANUAL]>");
		/*
		Este modo serve para leitura de posição e velocidade angular a partir dos valores do encoder (funções a definir no objetivo 2) e para manipulação contínua e direta do sinal de PWM (funções a definir no objetivo 3);
		Neste modo é possível:
			Solicitar pedidos de amostras (utilizador);
			Enviar comandos de PWM, seja por comando ou usando o teclado (utilizador);
			Alterar a posição 0 do motor - contador de posição inicializado (utilizador);
			Realizar o Enable do motor (utilizador);
		Relativamente ao comando do motor via PWM, apenas passa a ser atuado após a ativação do comando Enable do motor (EN=1). Um para cada sentido de rotação (para a ponte completa do BTS7960).
		A desativação do Enable do motor ou o pedido de paragem de leituras não deve fazer o programa sair do estado 2 (Modo de leitura/comando manual);
		Transição de estado:
			O estado 2 (Modo de leitura/comando manual) é ativado pelo acionamento do CS=2, apenas a partir do estado 1 (Modo de configuração);
			O estado 2 (Modo de leitura/comando manual) deve permanecer ativo até o estado 0 (modo de Reset) ou o estado 1 (Modo de configuração) ser selecionado. Não é possível passar para o estado 3 (modo automático)
		*/
//		if(last_CS != 2){
//
//		}
		while (CS==2){
			main_loop();
			if(CS!=2) break;
		}
		break;
	case 3:				//Auto
		print("\nAUTO---------------------");
		print("\n[AUTO]>");
		/*
		Este modo visa o teste do controlo do disco usando o controlo PID implementado. O programa a realizar para este estado será definido no objetivo 6;
		Após a ativação do estado 3 (Modo automático), o sistema deve ligar o Enable do motor (EN=1) e arrancar o teste em malha fechada;
		Este modo, depois de ativo, deverá permanecer em funcionamento independentemente de qualquer perturbação externa causada à carga mecânica;
		Caso o utilizador desative o Enable do motor (EN = 0), o motor deve parar e o sistema volta ao estado 1 (modo de configuração).
		Ao sair deste modo, o sinal de Enable do motor deve ser desativado;
		Transição de estado:
			O estado 3 (Modo automático) é ativado pelo acionamento do CS=3, a partir do estado 1 (Modo de configuração).
			O estado 3 (Modo automático) deve permanecer ativo até ao utilizador voltar a selecionar o estado 1 (Modo de configuração) ou o estado 0 (Modo de Reset). Se realizar o disable (EN=0) do motor deve passar para o estado 1 (Modo de configuração)
		*/
//		if(last_CS != 3){
//			EN = 1;
//		}
//		if(EN != 0){
//			CS=1;
//		}
		EN=1;
		while (CS==3){
			main_loop();
		}
		break;
	default:			//Safety
		CS = 0;
		break;
	}
}
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
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //print("Insira o comando.\n>");						//Apresenta esta mensagem inicial no termite para o utilizador
  //printf(">");
  start_scan(input);									//Início da receção pela usart3
  //uint16_t next_CCR = TIM3_CCR;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    //Define o valor inicial do Duty Cycle para 0 (0%)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);			//Ativa efetivamente a geração do sinal PWM no pino físico
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
	  state_machine();
//	  if(data_ready == 1){ 								//Verifica se a flag de receção foi ativada
//		  upperCase(input);								//Se foi, garante que todos os comandos recebidos ficam apenas em maiúsculas (por causa do Case Sensitivity)
//		  // 1. parse(): Divide a string em tokens e valida a sintaxe (Análise Léxica/Sintática)
//		  // 2. execute(): Recebe o resultado do parse e atua no hardware (Semântica/Execução)
//		  execute(parse(input, delim));
//
//		  //print("\n>");									// Imprime o prompt para indicar ao utilizador que o sistema está pronto para o próximo comando
//		  //Limpeza e Preparação para o próximo ciclo
//		  data_ready = 0;								//Reset da flag de receção
//		  memset(input, 0, MAX_CHAR); 					//Limpa o buffer de entrada para evitar resíduos de comandos anteriores (por segurança)
//		  // Reinício da escuta da UART
//		  if(start_scan(input) != HAL_OK){				// Tenta reativar o modo de receção
//			  __HAL_UART_CLEAR_OREFLAG(&huart3);		//Limpa o erro de Overrun para desbloquear o periférico
//			  start_scan(input);						// Segunda tentativa de arranque após limpeza do erro
//		  }
//	    }
//	  if (ov){											// Verifica se a flag de overflow está ativa
//	  		  ov = 0;									// Reset da flag (acknowledge)
//	  	  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	// Indicação visual que o código não "encravou" e o loop principal continua a correr
//	  	  }
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
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)	//Callback de Evento de Receção UART: Executado automaticamente pelo hardware
{

    if (huart->Instance == USART3){		//Verifica se a interrupção veio da USART3 (ligada ao ST-Link/Terminal do PC)
        input[Size] = '\0';				//Coloca o terminador nulo ('\0') exatamente na posição após o último caractere recebido
        data_ready = 1; 				//Ativa a flag de sinalização. O loop principal (while(1)) verá este '1' e saberá que pode começar a processar o comando.
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //Callback do Período do Timer: Executado quando o contador do Timer atinge o valor de Auto-Reload (ARR).
	if (htim == &htim3){ // Filtra para garantir que estamos a reagir apenas ao Timer 3
		ov = 1; // Sinaliza que o período de tempo decorreu (Overflow).
		//As ISR devem ser o mais curtas possível!
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

