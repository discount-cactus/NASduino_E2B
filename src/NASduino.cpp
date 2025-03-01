#include "Arduino.h"
#include "NASduino.h"



NAS::NAS(int addr[5], int data[8], int clk, int rw){
  _addr[5] = addr;
  _data[8] = data;
  _clk = clk;
  _rw = rw;
}

bool NAS::available(){
}

void NAS::read(){
}

void NAS::write(){
}

/*ParallelChip::ParallelChip(int addr[], int dq[], int adr_len, int dq_len, int ce, int we, int oe){
  _addrLen = adr_len;
  _dq_len = dq_len;
  _CE = ce;
  _WE = we;
  _OE = oe;

  locations = pow(2,_addrLen);

  for(uint8_t i=0; i < adr_len; i++){
    _addr[i] = addr[i];
  }
  for(uint8_t i=0; i < dq_len; i++){
    _dq[i] = dq[i];
  }
}

void ParallelChip::write(){

}

void ParallelChip::read(){

}

void ParallelChip::clear(){

}

void ParallelChip::dump(){

}
*/
