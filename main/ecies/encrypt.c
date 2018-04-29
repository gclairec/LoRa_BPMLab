
#include "encrypt.h"
#include "lora/lora.h"
#include <math.h>
//#define TIMER_KEY_GENERATION     500000                   //interval between each symetric key generation (in ms)
#define TIMER_MESURE             3000                     //interval between each sample (in ms)
//#define BUFF_SIZE_MAX            8    //240                 //size max in bit of the buffer to send. in bytes daw ana tita sally
//#define MSG_SIZE                24                         //size of one message to put in the buffer
//#define TEMP_SIZE                8
//#define PRESS_SIZE                8

#define ENCRYPT_DATA_SIZE			10

//Allow to generate the first symetric key before starting sampling
int FIRST_KEY=0;

//global variable shared between the different task
uint8_t key[32];
uint8_t extern_pubkey[64];
uint8_t wraped_key[132];
extern uint8_t privkey[32];


/*
 * \brief                            Fullfill global variable wrapped_buffer with iv || tag || encrypted_buffer
 *
 * \param    encrypted_buffer        The encrypted buffer to wrap before sending it
 *             iv                        Iv used for encryption
 *             tag                        Tag computed by the encryption
 *             wrapped_buffer            The buffer to fullfill
 *             wrap_size
 */
void wrap_buffer(uint8_t * encrypted_buffer, unsigned buffer_size,uint8_t * iv, uint8_t * tag,uint8_t * key,uint8_t * wrapped_buffer){
    memcpy(wrapped_buffer,iv,IV_SIZE);
     printf("\n wrapped_buffer: iv :");
     for(int i=0;i<IV_SIZE;i++){
                     printf("%x", wrapped_buffer[i]);
                 }
    memcpy(wrapped_buffer+IV_SIZE,tag,TAG_SIZE);
    printf("\n wrapped_buffer: iv+tag :");
     for(int i=0;i<IV_SIZE+TAG_SIZE;i++){
                     printf("%x", wrapped_buffer[i]);
                 }
    memcpy(wrapped_buffer+IV_SIZE+TAG_SIZE,encrypted_buffer,buffer_size);
     printf("\n wrapped_buffer: iv+tag+msg:");
     for(int i=0;i<ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE;i++){
                     printf("%x", wrapped_buffer[i]);
                 }
    printf("\n");
    memcpy(wrapped_buffer+IV_SIZE+TAG_SIZE+32, key,32);
    printf("\n wrapped_buffer: iv+tag+msg+key:");
     for(int i=0;i<ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE+32;i++){
                     printf("%x", wrapped_buffer[i]);
                 }
    printf("\n");
}


/*
 * \brief                            Fullfill global variable key with a AESGCM symetric key
 *
 */
void symetric_generation(){    //generates public key
    uint8_t ephemeral_pubkey[PUB_KEY_SIZE];
    int r=0;
    if((r=symetric_key_generation(key,extern_pubkey,ephemeral_pubkey))!=1){
        printf("error during symetric generation: %d\n",r);
    }
    return;
}

/*
 * \brief                            Encrypt the global variable key with ECIES cipher before sending it to the receiver
 *                                     Send a wrapped key of the form id_curve(3) || length(public_key) || ephemeral pubkey || tag || iv || ciphertext
 *
 * \param         param                Must be a pointer of the socket to send wrapped key
 *
 */
void ecc_key_task(void){
    memset(key,0,32);
    memset(wraped_key,0,32+4+PUB_KEY_SIZE+TAG_SIZE+IV_SIZE);
    symetric_generation();
    ECIES_encrypt_key(key,32,extern_pubkey,wraped_key);

    /*
     * Send the encrypted key thru LoRa
     */
    printf("\nsending encrypted key\n");
    bool res =lora_send(wraped_key);
    printf("\nsending encrypted key done!\n");

    //send(*sock,wraped_key,32+4+PUB_KEY_SIZE+TAG_SIZE+IV_SIZE,0);
    FIRST_KEY=1;
    memset(wraped_key,0,32+4+PUB_KEY_SIZE+TAG_SIZE+IV_SIZE);
    vTaskDelay(1000/portTICK_RATE_MS);
}


uint8_t* encrypt(uint8_t* dat_encrypt){

    //while(FIRST_KEY!=1){
        //wait for the first symetric key to be computed and sent
    //}
    int i = 0;
    //uint8_t buffer[BUFF_SIZE_MAX];

    uint8_t iv[IV_SIZE];
    uint8_t tag[TAG_SIZE];
    uint8_t encrypted_buffer[ENCRYPT_DATA_SIZE];
    uint8_t decrypted_buffer[ENCRYPT_DATA_SIZE];
    static uint8_t wraped_buffer[ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE];
    uint8_t encrypt_ecies[ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE];
    uint8_t decrypt_ecies[ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE];
    uint8_t local_key[32];
    random32(&key,32);    
    memcpy(local_key,key,32);
    printf("\nencrypt\n");
    printf("key: ");
    for(i=0;i<32;i++){    //print plain message
            printf("%x", local_key[i]);
        }
    printf("\n");

    //memcpy(buffer, data_param, BUFF_SIZE_MAX); //<-- i need to clarify this
    AESGCM(dat_encrypt,ENCRYPT_DATA_SIZE,MBEDTLS_ENCRYPT,key,iv,tag,encrypted_buffer);
        printf("\nreceived message\n");
        for(i=0;i<ENCRYPT_DATA_SIZE;i++){    //print plain message
            printf("%c", dat_encrypt[i]);
        }
        printf("\nencrypted message\n");
        for(i=0;i<ENCRYPT_DATA_SIZE;i++){
            printf("%x", (unsigned int)encrypted_buffer[i]);
        }
        wrap_buffer(encrypted_buffer,IV_SIZE+TAG_SIZE+ENCRYPT_DATA_SIZE,iv,key,tag,wraped_buffer);

        //send(*sock,wraped_buffer,BUFF_SIZE_MAX+IV_SIZE+TAG_SIZE,0);

    //ECIES_encrypt_key(wraped_buffer, ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE, extern_pubkey, encrypt_ecies);

    AESGCM(encrypted_buffer,ENCRYPT_DATA_SIZE,MBEDTLS_DECRYPT,key,iv,tag,decrypted_buffer);
        printf("\ndecrypted message\n");
        for(i=0;i<ENCRYPT_DATA_SIZE;i++){
            printf("%c", decrypted_buffer[i]);
        }
    //ECIES_decrypt_key(encrypt_ecies, ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE, extern_pubkey, privkey, decrypt_ecies);

    //memset(buffer,0,ENCRYPT_DATA_SIZE);
    memset(encrypted_buffer,0,ENCRYPT_DATA_SIZE);
    memset(decrypted_buffer,0, ENCRYPT_DATA_SIZE);
    memset(iv,0,IV_SIZE);
    memset(tag,0,TAG_SIZE);
    //memset(wraped_buffer,0,ENCRYPT_DATA_SIZE+IV_SIZE+TAG_SIZE);
    //vTaskDelay(TIMER_MESURE/portTICK_RATE_MS);

    /*
     * Return the encryted data in a wrapped buffer (42 bytes total)
     */
    return(wraped_buffer);
}

uint8_t *data_encrypt(uint8_t *data)
{
    // generate key and send
    ecc_key_task();
    //encrypt data and send
    printf("\ndata encrypting-->\n");
    uint8_t *encrypted = encrypt(data);
    printf("\n");
    for(int i=0;i<IV_SIZE+TAG_SIZE+ENCRYPT_DATA_SIZE;i++){
                   printf("%x", (unsigned int)encrypted[i]);
               }
    printf("\ndata encrypting<--finished\n");
    return(encrypted);
}






