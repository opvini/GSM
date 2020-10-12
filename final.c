
#include <18f452.h>


#use delay(clock=20000000) 
#fuses HS
#use rs232(baud=9600, parity=N, bits=8, xmit=PIN_C6, rcv=PIN_C7, ERRORS)

#define GSM_ATS3_FIXO 13
#define GSM_ATS4_FIXO 10

#priority RDA, timer0

#define use_portb_lcd true
#include <aev_lcd.c>
#include <aev_string.h>


void intrda();
int1 gsm_exe(char *dado, long int max_time_out, int max_tentativas, char *resp_esperada);
void gsm_recebe_resposta();
void gsm_init();
void gsm_erro_init();
void gsm_iniciou_adm();
void gsm_iniciou();
void gsm_executa_resposta();
void gsm_erro_interrupt();
int  gsm_verifica_sinal();
void gsm_mostra_sinal();
int1 gsm_apaga_sms(int mem);
void gsm_apaga_todas_sms();
void gsm_envia_sms();
void gsm_limpa_buffer_serial();
int1 gsm_verifica_comunicacao();
int1 gsm_le_sms(int mem);
void gsm_open_gprs();
void gsm_close_gprs();
void gsm_gprs_send();

void intTimer0();
void habilita_interrupts();
void desabilita_interrupts();
char getc_timeout();
void gsm_trata_sms(int mem);
void gsm_leu_sms();
void gsm_erro_le_sms();
void gsm_apagou_sms(int mem);
void gsm_erro_apaga_sms(mem);
void gsm_apagou_todas_sms();
int gsm_get_mem_sms();

void processo_gsm();


struct gsm_sms_estrutura{
   char numero[15];
   char data[25];
   char mensagem[170];
   char status[15];
   int  mem;
} gsm_sms;


char     gsm_resposta[250], gsm_resp_esperada[10], gsm_cmd[60], gsm_ip[16];
long int gsm_max_time_out = 500;
int      gsm_ats4=10, gsm_ats3=13, gsm_max_seq=2, cont_timer0=0;

int1 gsm_flag_terminou=0, gsm_flag_timeout, gsm_flag_recebendo_ligacao=0;
int1 gsm_flag_interrupt=0,gsm_flag_enviado_sms=0;
int1 gsm_iniciado=0, gsm_flag_gprs_send=0;


void intTimer0();


// sinalizadores que ocorreu o tempo
int1 flag_timer_1s=0, flag_timer_5s=0, flag_timer_10s=0, flag_timer_30s=0, flag_timer_60s=0;

// contadores para indicar que ocorreu o tempo
int cflag_timer_1s=0, cflag_timer_5s=0, cflag_timer_10s=0, cflag_timer_30s=0, cflag_timer_60s=0;




/////////////////////////////////////////////////////////////////////////// ESTRUTURA SERIAL

 
// GETC sem travar o programa
// com tempo limite <gsm_max_time_out>

char getc_timeout() {
   long int gsm_cont_tempo1=0, gsm_cont_tempo2=0;
   int1 gsm_flag_chegou=0;
   char tmp_a;   

     for(gsm_cont_tempo1=0; (!gsm_flag_chegou && gsm_cont_tempo1 <= 100); gsm_cont_tempo1++){
        for(gsm_cont_tempo2=0; gsm_cont_tempo2 <= gsm_max_time_out; gsm_cont_tempo2++ ){
        
            if( kbhit() ){
               tmp_a           = getc();
               gsm_flag_chegou = 1;
               break;
            }  
            else{
               delay_us(10);
               gsm_flag_chegou = 0;
               continue;
            }
            
        }
     }
     
     if( !gsm_flag_chegou )  return(0);
     else                    return(tmp_a);
     
}


// responsavel por receber os dados
// sem travar o programa
// fica esperando no maximo <max_timeout> algum dado
// porem se chegou o sinalizador de fim de mensage <ats3><ats4> ai sai

void gsm_recebe_resposta(){

   char gsm_a, gsm_last_char='a';
   int gsm_cont_seq=0, gsm_cont_char=0;

   strcpy( gsm_resposta, "");
   gsm_flag_terminou = 0;
   gsm_flag_timeout  = 0;


   // a mensagem acaba na segunda sequencia
   // <ats3><ats4> recebida, ou seja
   // no caso na segunda seq. \r\3
   // enquanto ela nao chega, a mensagem nao terminou
   // a nao ser que o tempo limite tenha passado
  
   
   while(1){
   
     
     // espera receber algum dado em no maximo <gsm_max_time_out> milisegundos
     // se nao chegou eh sinal que travou
     
     gsm_a = getc_timeout();
     
     // travou no GETC 
     // passou do tempo limite
     if(gsm_a == 0){
         gsm_flag_terminou = 1;
         gsm_flag_timeout  = 1;
         gsm_resposta[ gsm_cont_char++ ] = '\0';
         return;
     }
     
     // manipula a resposta do modem
     
     if( gsm_a == gsm_ats4 && gsm_last_char == gsm_ats3 ) gsm_cont_seq++;
     

     // despresa ats3 e ats4 que sao de configuracoes
     // aqui esta a informacao a ser recebida
     if(gsm_a != gsm_ats3 && gsm_a != gsm_ats4){
         gsm_resposta[ gsm_cont_char++ ] = gsm_a;
     }
     
     
     // enviando SMS, a resposta eh "> "
     if(gsm_flag_enviado_sms && gsm_resposta[0] == '>' && gsm_resposta[1] == ' '){
        gsm_resposta[ gsm_cont_char++ ] = '\0';
        gsm_flag_terminou     = 1;
        gsm_flag_enviado_sms  = 0;
        gsm_flag_timeout      = 0;
        return;
     }
     
     
     // se recebeu um RING
     // muda o <gsm_max_seq> por causa do CLIP
     if(
         gsm_flag_interrupt && gsm_cont_char == 4 &&
         gsm_resposta[0] == 'R' && gsm_resposta[1] == 'I' && gsm_resposta[2] == 'N' && gsm_resposta[3] == 'G' 
       )
       gsm_max_seq = 4;
       
         
      
     // chegou a sequencia indicando final de mensagem
     // acabou a transmissao
     if( gsm_cont_seq >= gsm_max_seq ){
        gsm_resposta[ gsm_cont_char++ ] = '\0';
        gsm_flag_terminou = 1;
        gsm_flag_timeout  = 0;
        return;
     }
     else gsm_last_char = gsm_a;
      
   } // fim while


}// gsm_recebe_interrupt



// envia um dado para a porta serial
// espera receber dados por um tempo limite e um limite de tentativas
// caso receba retorna 1 caso contrario retorna 0

int1 gsm_exe(char *dado, long int max_time_out, int max_tentativas, char *resp_esperada){

   long int tmp_timeout;
   int gsm_cont_tents = 0;
   int1 resposta_ok   = 0;
   
   gsm_flag_terminou = 0;
   tmp_timeout       = gsm_max_time_out;
   gsm_max_time_out  = max_time_out;
   
   disable_interrupts( INT_RDA );
  
   
   // enquanto nao tentou o limite maximo
   // no tempo limite maximo
   
   while( !gsm_flag_terminou && (++gsm_cont_tents) <= max_tentativas ){
      
      // envia o dado
      // e espera receber uma resposta completa
   
      printf("%s%c", dado, gsm_ats3);
      gsm_recebe_resposta();
      
      
      // se obteve resposta
      // e espera receber algo
      // faz o teste e sinaliza em <resposta_ok>
      
      if(strlen(resp_esperada) > 0)
         resposta_ok = !strcmp( gsm_resposta, resp_esperada);
      else if(gsm_flag_terminou && strlen(resp_esperada) <= 0)
         resposta_ok = 1;
      else
         resposta_ok = 0;
         
      
      // se a mensagem nao chegou
      // num tempo maximo exigido
      // ou a resposta experada nao foi o desejado
      // envia novamente o comando
      
      if( gsm_flag_terminou && gsm_flag_timeout ){
         gsm_flag_terminou = 0;
         printf( lcd_putc, "\fTentou %d", gsm_cont_tents);
         delay_ms(1000);       
         continue;
      }
      
      else if( gsm_flag_terminou && !gsm_flag_timeout && !resposta_ok ){
         gsm_flag_terminou = 0;
         printf( lcd_putc, "\fTentou %d", gsm_cont_tents);
         delay_ms(1000);
         printf("%s%c", dado, gsm_ats3);
         continue;
      }
      
      else if( gsm_flag_terminou && !gsm_flag_timeout && resposta_ok ){
         gsm_flag_terminou = 0;
         gsm_max_seq       = 2;
         gsm_max_time_out  = tmp_timeout;
         enable_interrupts( INT_RDA );
         return(1);
      }
      
      
   }// while
   
   
   // tentou o maximo de vezes enviar o comando
   // e nao obteve sucesso
   // logo o modem esta travado ou sem comunicacao
   
   gsm_flag_terminou = 0;
   gsm_max_seq       = 2;
   gsm_max_time_out  = tmp_timeout;
   
   enable_interrupts( INT_RDA );   
   return(0);
}


//////////////////////////////////////////////////////////////////////////////////////





void gsm_executa_interrupt(){
   
   char gsm_resposta_esperada[15], gsm_resposta_esperada_tmp[15];
   int mem_sms;
   
   strcpy(gsm_resposta_esperada, "RING");
   substr(0,4,gsm_resposta,gsm_resposta_esperada_tmp);
   if( strcmp(gsm_resposta_esperada, gsm_resposta_esperada_tmp) == 0 ){
      gsm_flag_recebendo_ligacao = 1;
      substr(12,11,gsm_resposta,gsm_resposta_esperada_tmp);
      printf(lcd_putc, "\fChamando...\n%s", gsm_resposta_esperada_tmp);
      return;
   }
   
   strcpy(gsm_resposta_esperada, "NO CARRIER");
   if( strcmp(gsm_resposta_esperada, gsm_resposta) == 0 ){
      gsm_flag_recebendo_ligacao = 0;
      printf(lcd_putc, "\fDesligou");
      delay_ms(1000);
      return;
   }
   
   strcpy(gsm_resposta_esperada, "CALL READY");
   if( strcmp(gsm_resposta_esperada, gsm_resposta) == 0 ){
      printf(lcd_putc, "\fLigacao perdida");
      delay_ms(1000);
      return;
   }
   
   strcpy(gsm_resposta_esperada, "+CMTI:");
   substr(0,6,gsm_resposta,gsm_resposta_esperada_tmp);
   if( strcmp(gsm_resposta_esperada, gsm_resposta_esperada_tmp) == 0 ){
      mem_sms = gsm_get_mem_sms();
      printf(lcd_putc, "\fRecebeu SMS: %d",mem_sms);
      delay_ms(1000);
      gsm_le_sms(mem_sms);
      gsm_apaga_sms(mem_sms);
      gsm_envia_sms();
      return;
   }
   
   strcpy(gsm_resposta_esperada, "OK");
   if( strcmp(gsm_resposta_esperada, gsm_resposta_esperada_tmp) == 0 ){
      printf(lcd_putc, "\fModem\nConectado");
      return;
   }

} //gsm_executa_resposta()



int1 gsm_verifica_comunicacao(){
   gsm_max_seq = 2;
   strcpy(gsm_cmd, "AT");
   strcpy( gsm_resp_esperada, "OK");
   if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ) return(1);
   else return(0);
}



void processo_gsm(){


      // recebeu dados sem solicitar
      // e os dados jah estao salvos: interrupcao
      
      if(gsm_flag_terminou && !gsm_flag_timeout && gsm_flag_interrupt){
         gsm_flag_terminou  = 0;
         gsm_flag_interrupt = 0;
         gsm_executa_interrupt();     
      }
      
      // reinicializa pois houve interrupcao
      // com time_out
      
      else if(gsm_flag_terminou && gsm_flag_timeout && gsm_flag_interrupt){
         gsm_flag_terminou  = 0;
         gsm_flag_interrupt = 0;
      }
      
      
      // verifica se o modem esta conectado normalmente
      // se nao tiver processando nenhuma resposta, nem recebendo ligacao
      
      else if( !gsm_flag_terminou && !gsm_flag_recebendo_ligacao && !gsm_flag_interrupt && flag_timer_5s ){
         flag_timer_5s = 0;
         output_high(PIN_C2);
         
         if( !gsm_verifica_comunicacao() ){
            printf(lcd_putc, "\fModem\nDesconectado");
            gsm_gprs_send();
            
            strcpy(gsm_cmd, "AT+CIPCLOSE");
            gsm_exe(gsm_cmd, 500, 5, gsm_resp_esperada);
            
         }
         else printf(lcd_putc, "\fModem\nConectado");
         
         output_low(PIN_C2);
      }
      
}


int gsm_get_mem_sms(){
   int i, totMsg;
   char mem_sms[3];
   totMsg = strlen(gsm_resposta);
   for(i=(totMsg-1);i>0;i--) if( gsm_resposta[i] == ',' ) break;
   substr(i+1, totMsg-2, gsm_resposta, mem_sms);
   return strtoint(mem_sms);
}


// retorna o sinal da operadora
// 0: problema na comunicacao
// sem sinal 1, baixo 2, bom 3, excelente 4

int gsm_verifica_sinal(){
   char gsm_tmp_str[5], gsm_str_esperada[5];
   int gsm_sinal=0;
   
   gsm_max_seq = 4;
   
   strcpy(gsm_cmd, "AT+CSQ");
   strcpy( gsm_resp_esperada, "");
   
   if( gsm_exe(gsm_cmd, 1000, 5, gsm_resp_esperada) ){
   
      strcpy(gsm_str_esperada, "+CSQ");
      substr(0,4,gsm_resposta,gsm_tmp_str);
      
      if( strcmp(gsm_tmp_str, gsm_str_esperada) == 0 ){
         substr(6,2,gsm_resposta,gsm_tmp_str);        // guarda o sinal
         substr(9,1,gsm_resposta,gsm_str_esperada);   // guarda o erro
         
         gsm_sinal = strtoint(gsm_tmp_str);
         
         // Sinal = 1, 2, 3 ou 4
         // sendo sem sinal, baixo, bom, excelente
         if( gsm_sinal >= 19 )                        gsm_sinal = 4;
         else if( gsm_sinal < 19 && gsm_sinal >= 14 ) gsm_sinal = 3;
         else if( gsm_sinal < 14 && gsm_sinal >= 9 )  gsm_sinal = 2;
         else gsm_sinal = 1;
         
         return(gsm_sinal);
      }
      else return(0);
      
   }
   else return(0);
}


void gsm_envia_sms(){

   gsm_flag_enviado_sms = 1;
   char tmp_resp[3];
   
   strcpy( gsm_resp_esperada, "");
   
   strcpy(gsm_cmd, "AT+CMGS=\"+553188508881\"");   
   if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
   
      gsm_max_seq = 4;
      
      sprintf(gsm_cmd, "Recebi sua mensagem.%c", 26);     
      if( gsm_exe(gsm_cmd, 10000, 5, gsm_resp_esperada) ){
      
         substr(-2,2,gsm_resposta,tmp_resp);
         strcpy( gsm_resp_esperada, "OK");
      
         if( strcmp(tmp_resp,gsm_resp_esperada) == 0 ){
         
            printf(lcd_putc,"\fSms enviada.");
            delay_ms(1000);
            
         }
         else printf(lcd_putc,"\ferro: .%s.", tmp_resp);
         
      }
      
   }
}


int1 gsm_apaga_sms(int mem){  
   gsm_max_seq = 2;
   
   sprintf(gsm_cmd, "AT+CMGD=%d", mem);
   strcpy( gsm_resp_esperada, "OK");
   
   if( gsm_exe(gsm_cmd, 1000, 5, gsm_resp_esperada) ){
      gsm_apagou_sms(mem);
      return(1);
   }
   else{
      gsm_erro_apaga_sms(mem);
      return(0);
   }
}



void gsm_apaga_todas_sms(){
   int i;
   for(i=1; i<=20; i++) gsm_apaga_sms(i);
   gsm_apagou_todas_sms();
}


int1 gsm_le_sms(int mem){
   char gsm_resposta_esperada[3], gsm_resposta_esperada_tmp[3];
   
   gsm_max_seq = 5;
   
   sprintf(gsm_cmd, "AT+CMGR=%d", mem);
   strcpy( gsm_resp_esperada, "");
   
   if( gsm_exe(gsm_cmd, 2000, 5, gsm_resp_esperada) ){
   
      strcpy(gsm_resposta_esperada, "OK");
      substr(-2, 2, gsm_resposta, gsm_resposta_esperada_tmp);
      
      if( strcmp(gsm_resposta_esperada, gsm_resposta_esperada_tmp) == 0 ){  
         gsm_trata_sms(mem);
         gsm_leu_sms();
         return(1);
      }
      else{
         gsm_erro_le_sms();
         return(0);
      }
      
   }
   else{
      gsm_erro_le_sms();
      return(0);
   }
}


// salva os dados da SMS na estrutura <gsm_sts>

void gsm_trata_sms(int mem){
   int i, ct=0, ini;
   char procurando = ',';
   
   for(i=0; ct<=5; i++){
      if( gsm_resposta[i] == procurando ){
         ct++;
               
         // status
         if( ct == 1 ){
            ini = i;
            substr(8, (i-9), gsm_resposta, gsm_sms.status);
         }
         
         // numero
         else if( ct == 2 ){
            substr((ini+2), (i-(ini+3)), gsm_resposta, gsm_sms.numero);
            ini = i;
         }
         
         
         // data e hora
         else if( ct == 4 ) procurando = '"';         
         else if( ct == 5 ){
            substr((ini+3), (i-(ini+3)), gsm_resposta, gsm_sms.data);
            ini = i;
         }
         
      } // if
   } // for
   
   substr((ini+1), strlen(gsm_resposta)-(ini+3), gsm_resposta, gsm_sms.mensagem);
   gsm_sms.mem = mem;
   
}


void gsm_leu_sms(){
         
         printf(lcd_putc, "\fstatus\n%s", gsm_sms.status);
         delay_ms(1000);
         
         printf(lcd_putc, "\fnumero\n%s", gsm_sms.numero);
         delay_ms(1000);
         
         printf(lcd_putc, "\fdata\n%s", gsm_sms.data);
         delay_ms(1000);
         
         printf(lcd_putc, "\fmensagem\n%s", gsm_sms.mensagem);
         delay_ms(1000);
}



//********************* GPRS

/*
 
AT+CIPSTATUS
OK

STATE: IP STATUS

AT+CIPSTART="TCP","www.vinicius.info","80"
OK

CONNECT OK
AT+CIPSEND
*/

void gsm_open_gprs(){

   gsm_close_gprs();

   strcpy( gsm_resp_esperada, "OK");
   strcpy(gsm_cmd, "AT+CGATT=1");
   if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
   
      printf(lcd_putc, "\fAT+CGATT\n%s", gsm_resposta);
      delay_ms(500);
      
      strcpy(gsm_cmd, "AT+CGDCONT=1,\"IP\",\"gprs.oi.com.br\"");
      if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
      
         printf(lcd_putc, "\f%s\n%s", gsm_cmd, gsm_resposta);
         delay_ms(500);
   
         strcpy(gsm_cmd, "AT+CDNSCFG=\"200.222.108.241\"");
         if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
         
            printf(lcd_putc, "\f%s\n%s", gsm_cmd, gsm_resposta);
            delay_ms(500);
         
            strcpy(gsm_cmd, "AT+CSTT=\"gprs.oi.com.br\",\"oiwap\",\"oioioi\"");
            if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
               
               printf(lcd_putc, "\f%s\n%s", gsm_cmd, gsm_resposta);
               delay_ms(500);
            
               //printf(lcd_putc,"\fConectando...");
               strcpy(gsm_cmd, "AT+CIICR");
               if( gsm_exe(gsm_cmd, 65000, 1, gsm_resp_esperada) ){
               
                  printf(lcd_putc, "\f%s\n%s", gsm_cmd, gsm_resposta);
                  delay_ms(500);
               
                  strcpy( gsm_resp_esperada, "");
                  strcpy(gsm_cmd, "AT+CIFSR");
                  if( gsm_exe(gsm_cmd, 300, 5, gsm_resp_esperada) ){

                     strcpy(gsm_ip, gsm_resposta);
                     printf(lcd_putc, "\fIP: %s", gsm_ip);
                     delay_ms(3000);
                     
                     strcpy( gsm_resp_esperada, "OK");
                     strcpy(gsm_cmd, "AT+CIPHEAD=1");
                     if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
                     
                        strcpy(gsm_cmd, "AT+CDNSORIP=0");
                        if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
                        
                           printf(lcd_putc, "\fChegou no fim");
                           gsm_gprs_send();
                        
                     
                        } // AT+CDNSORIP
                        else { printf(lcd_putc, "\ferro 1\n%s", gsm_resposta); delay_ms(3000); }
                     
                     } // AT+CIPHEAD
                     else { printf(lcd_putc, "\ferro 2\n%s", gsm_resposta); delay_ms(3000); }
                        
                  } // AT+CIFSR
                  else { printf(lcd_putc, "\ferro 3\n%s", gsm_resposta); delay_ms(3000); }
               
               } // AT+CIICR
               else { printf(lcd_putc, "\ferro 4\n%s", gsm_resposta); delay_ms(3000); }
            
            } // AT+CSTT
            else { printf(lcd_putc, "\ferro 5\n%s", gsm_resposta); delay_ms(3000); }
   
         } // AT+CDNSCFG
         else { printf(lcd_putc, "\ferro 6\n%s", gsm_resposta); delay_ms(3000); }
         
      } // AT+CGDCONT
      else { printf(lcd_putc, "\ferro 7\n%s", gsm_resposta); delay_ms(3000); }
      
   } // AT+CGATT
   else { printf(lcd_putc, "\ferro 8\n%s", gsm_resposta); delay_ms(3000); }

}


void gsm_close_gprs(){
   strcpy( gsm_resp_esperada, "");
   
   strcpy(gsm_cmd, "AT+CIPCLOSE");
   gsm_exe(gsm_cmd, 500, 5, gsm_resp_esperada);
   
   strcpy(gsm_cmd, "AT+CIPSHUT");
   gsm_exe(gsm_cmd, 500, 5, gsm_resp_esperada);
 
}

void gsm_gprs_send(){

   strcpy( gsm_resp_esperada, "");
   strcpy(gsm_cmd, "AT+CIPSTART=\"TCP\",\"201.78.104.249\",\"8080\"");
   if( gsm_exe(gsm_cmd, 20000, 5, gsm_resp_esperada) ){
      
      printf(lcd_putc, "\f%s", gsm_resposta);
      delay_ms(3000);
      
         strcpy(gsm_cmd, "AT+CIPSEND");
         if( gsm_exe(gsm_cmd, 100, 5, gsm_resp_esperada) ){
      
         printf(lcd_putc, "\f%s", gsm_resposta);
         delay_ms(1000);
         
            sprintf(gsm_cmd,"FEIO DO GAS%c",26);
            if( gsm_exe(gsm_cmd, 10000, 5, gsm_resp_esperada) ){
      
               printf(lcd_putc, "\fTerminou\n%s", gsm_resposta);
               delay_ms(1000);
            
            }
            
         }
   }
   
}


//**************************



void gsm_erro_le_sms(){
   printf(lcd_putc, "\fErro ao ler sms");
}


void gsm_apagou_sms(int mem){
   printf(lcd_putc, "\fapagou sms %d", mem);
   delay_ms(500);
}


void gsm_erro_apaga_sms(mem){
   printf(lcd_putc, "\ferro apaga sms %d", mem);
}


void gsm_apagou_todas_sms(){
   printf(lcd_putc, "\fApagou todas sms");
}

void gsm_erro_init(){
   printf(lcd_putc, "\fErro ao iniciar\no modem.");
}


void gsm_iniciou(){
   printf(lcd_putc, "\fModem iniciado.");
}

void gsm_erro_interrupt(){
   printf(lcd_putc, "\fErro ao receber\nalgum dado.");
}




void gsm_mostra_sinal(){
   int gsm_sinal=0;
   gsm_sinal = gsm_verifica_sinal();
         
   switch(gsm_sinal){
      case 0: gsm_erro_init(); break;
      case 1: printf(lcd_putc, "\fSinal: sem sinal");   break;
      case 2: printf(lcd_putc, "\fSinal: baixo");       break;
      case 3: printf(lcd_putc, "\fSinal: bom");         break;
      case 4: printf(lcd_putc, "\fSinal: excelente");   break;
   }
}


void gsm_iniciou_adm(){
   gsm_iniciado = 1;
   printf(lcd_putc, "\fModem iniciado!");
   delay_ms(1000);
   gsm_mostra_sinal();
   delay_ms(1000);
}




void gsm_init(){
   
   if(!gsm_iniciado){
      gsm_ats3 = 13;
      gsm_ats4 = 10;
   }


   gsm_max_seq = 2;
   strcpy( gsm_resp_esperada, "");
   strcpy(gsm_cmd, "AT");
   if( gsm_exe(gsm_cmd, 100, 20, gsm_resp_esperada) ){
      
      strcpy(gsm_cmd, "ATE0");
      if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ){

         sprintf(gsm_cmd, "ATS3=%d", GSM_ATS3_FIXO);         
         if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ){
      
               gsm_ats3 = GSM_ATS3_FIXO;
            
               sprintf(gsm_cmd, "ATS4=%d", GSM_ATS4_FIXO);         
               if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ){
      
                  gsm_ats4 = GSM_ATS4_FIXO;
                           
                  strcpy(gsm_cmd, "AT+CMGF=1");
                  if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ){
               
                     strcpy(gsm_cmd, "AT+CLIP=1");
                     if( gsm_exe(gsm_cmd, 1000, 10, gsm_resp_esperada) ){
                     
                           strcpy(gsm_cmd, "AT");
                           strcpy(gsm_resp_esperada, "OK");
                           if( gsm_exe(gsm_cmd, 100, 10, gsm_resp_esperada) ){      
                     
                              // INICIOU
                              // faz algumas configuracoes
                              gsm_iniciou_adm();
                           
                           } // AT final
                           else gsm_erro_init();
                     
                     } // AT+CLIP=1
                     else gsm_erro_init();
               
                  } // AT+CMGF=1
                  else gsm_erro_init();
                  
               
               } // ATS4
               else gsm_erro_init();
         
         } // ATS3
         else gsm_erro_init();
      
      
      } // ATE0
      else gsm_erro_init();
   

   } // AT
   else gsm_erro_init();
   
}





void main(){ 
  
  // Configurando o timer0 para sabe se a mensagem serial acabou (100ms)
  // divide 5MHz por 32 chegando a 156250Hz
  // divide novamente por 15625 chegando a uma frequencia final de 10Hz
  // logo, a cada 100 interrupcoes, temos um intervalo de 10s
  // 65536 - 15625 = 49911 que deve ser o valor inicial da contagem
  
  setup_timer_0( RTCC_INTERNAL | RTCC_DIV_32 );
  set_timer0(49911);
  
  
  lcd_init();
  printf(lcd_putc, "\fIniciando...\n");
  delay_ms(1000);
  
  gsm_init();  
  //gsm_apaga_todas_sms();
  delay_ms(3000);
  //printf(lcd_putc,"\fLigando GPRS");
  //gsm_open_gprs();
  //gsm_envia_sms();
  
  enable_interrupts( INT_RDA );
  enable_interrupts( INT_TIMER0 );
  enable_interrupts( GLOBAL );
  
  
  while(1){
      processo_gsm();      
  }
  
}


#INT_RDA
void intrda(){

   // desabilita as interrupcoes
   // para receber toda a informacao serial
   
   disable_interrupts( INT_RDA );
   output_high(PIN_C3);   
   
   gsm_flag_interrupt = 1;
   gsm_max_seq        = 2;
   
   gsm_recebe_resposta(); 
   
   output_low(PIN_C3);   
   clear_interrupt( INT_RDA );
   enable_interrupts( INT_RDA );

}



// TIMER 0
// frequencia de 10Hz
// sinaliza 1s, 5s, 10s, 30s e 60s

#int_timer0
void intTimer0(){
   
   disable_interrupts( int_timer0 );
  
   cont_timer0++;
   set_timer0(49911);
   
   
   // 1 segundo
   if( cont_timer0 == 10 ){
      cont_timer0   = 0;
      flag_timer_1s = 1;
      cflag_timer_1s++;
   }
   
   // 5 segundos
   if( cflag_timer_1s == 5 ){
      cflag_timer_1s = 0;
      flag_timer_5s  = 1;
      cflag_timer_5s++;
   }
  
   // 10 segundos
   if( cflag_timer_5s == 2 ){
      cflag_timer_5s = 0;
      flag_timer_10s = 1;
      cflag_timer_10s++;
   }
   
   // 30 segundos
   if( cflag_timer_10s == 3 ){
      cflag_timer_10s = 0;
      flag_timer_30s  = 1;
      cflag_timer_30s++;
   }
   
   // 60 segundos
   if( cflag_timer_30s == 2 ){
      cflag_timer_30s = 0;
      flag_timer_60s  = 1;
      cflag_timer_60s++;
   }
   
   enable_interrupts( int_timer0 );
   
}
