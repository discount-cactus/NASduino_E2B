#ifndef NASduino_h
#define NASduino_h

#include "Arduino.h"

class NAS{
  // user-accessible "public" interface
  public:
    NAS(int addr[5], int data[8], int clk, int rw);
    bool available();
    void read();
    void write();

    int _addr[5];
    int _data[8];
    int _clk;
    int _rw;

  // library-accessible "private" interface
  private:
};

/*class ParallelChip{
  // user-accessible "public" interface
  public:
    ParallelChip(int addr[], int dq[], int adr_len, int dq_len, int ce, int we, int oe);
    void write();
    void read();
    void clear();
    void dump();

    int _addrLen;
    int _dq_len;
    //int _CE;
    //int _WE;
    //int _OE;
    int locations;

    int _addr[];
    int _dq[];

  // library-accessible "private" interface
  private:
    int _CE;
    int _WE;
    int _OE;
};*/

#endif    //NASduino_h
