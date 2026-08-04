#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* --- копии чистой логики из protocol_crsf.c / protocol_sbus.c --- */
static uint8_t crc8(const uint8_t*d,size_t n){uint8_t c=0;for(size_t i=0;i<n;i++){c^=d[i];for(int b=0;b<8;b++)c=(c&0x80)?(uint8_t)((c<<1)^0xD5):(uint8_t)(c<<1);}return c;}

static size_t crsf_build(const uint16_t ch[16],uint8_t*o){
  uint8_t pl[22]={0};uint32_t bb=0;int bc=0;size_t k=0;
  for(int i=0;i<16;i++){bb|=((uint32_t)(ch[i]&0x7FF))<<bc;bc+=11;while(bc>=8){pl[k++]=bb&0xFF;bb>>=8;bc-=8;}}
  o[0]=0xC8;o[1]=24;o[2]=0x16;memcpy(o+3,pl,22);o[25]=crc8(o+2,23);return 26;}

static void crsf_unpack(const uint8_t*pl,uint16_t*out){
  uint32_t bb=0;int bc=0,c=0;
  for(int i=0;i<22&&c<16;i++){bb|=((uint32_t)pl[i])<<bc;bc+=8;
    while(bc>=11&&c<16){out[c++]=bb&0x7FF;bb>>=11;bc-=11;}}}

static size_t sbus_build(const uint16_t ch[16],bool fs,uint8_t*o){
  memset(o,0,25);o[0]=0x0F;uint32_t bb=0;int bc=0;size_t k=1;
  for(int i=0;i<16;i++){bb|=((uint32_t)(ch[i]&0x7FF))<<bc;bc+=11;while(bc>=8){o[k++]=bb&0xFF;bb>>=8;bc-=8;}}
  o[23]=fs?0x08:0x00;o[24]=0x00;return 25;}

static void sbus_unpack(const uint8_t*f,uint16_t*out){
  const uint8_t*pl=f+1;uint32_t bb=0;int bc=0,c=0;
  for(int i=0;i<22&&c<16;i++){bb|=((uint32_t)pl[i])<<bc;bc+=8;
    while(bc>=11&&c<16){out[c++]=bb&0x7FF;bb>>=11;bc-=11;}}}

static size_t mav_total(const uint8_t*b,size_t have){
  if(have<1)return 0;
  if(b[0]==0xFE){if(have<2)return 0;return 6+b[1]+2;}
  if(b[0]==0xFD){if(have<3)return 0;size_t s=10+b[1]+2;if(b[2]&1)s+=13;return s;}
  return 0;}

int fails=0;
#define CHECK(c,msg) do{ if(c){printf("  PASS  %s\n",msg);} else {printf("  FAIL  %s\n",msg);fails++;} }while(0)

int main(void){
  printf("== CRSF ==\n");
  /* Известный вектор: CRC8/DVB-S2 от "123456789" = 0xBC */
  CHECK(crc8((const uint8_t*)"123456789",9)==0xBC,"CRC8 DVB-S2 check value 0xBC");

  uint16_t in[16],out[16];
  for(int i=0;i<16;i++) in[i]=172+i*100;      /* типовой диапазон CRSF 172..1811 */
  uint8_t fr[26]; crsf_build(in,fr);
  CHECK(fr[0]==0xC8&&fr[1]==24&&fr[2]==0x16,"frame header C8/24/0x16");
  CHECK(crc8(fr+2,23)==fr[25],"self CRC verifies");
  crsf_unpack(fr+3,out);
  int ok=1;for(int i=0;i<16;i++) if(in[i]!=out[i]) ok=0;
  CHECK(ok,"16ch pack->unpack roundtrip");
  uint16_t mx[16];for(int i=0;i<16;i++)mx[i]=2047;
  crsf_build(mx,fr);crsf_unpack(fr+3,out);
  ok=1;for(int i=0;i<16;i++)if(out[i]!=2047)ok=0;
  CHECK(ok,"max value 2047 (11-bit) roundtrip");

  printf("== S.Bus ==\n");
  uint8_t sf[25]; sbus_build(in,false,sf);
  CHECK(sf[0]==0x0F&&sf[24]==0x00,"start 0x0F / end 0x00");
  sbus_unpack(sf,out);
  ok=1;for(int i=0;i<16;i++)if(in[i]!=out[i])ok=0;
  CHECK(ok,"16ch pack->unpack roundtrip");
  sbus_build(in,true,sf);
  CHECK((sf[23]&0x08)!=0,"failsafe flag set");

  printf("== MAVLink framing ==\n");
  uint8_t v1[]={0xFE,9,0,1,1,0};
  CHECK(mav_total(v1,6)==17,"v1 HEARTBEAT total len = 17");
  uint8_t v2[]={0xFD,9,0,0,0,1,1,0,0,0};
  CHECK(mav_total(v2,10)==21,"v2 HEARTBEAT total len = 21");
  uint8_t v2s[]={0xFD,9,0x01,0,0,1,1,0,0,0};
  CHECK(mav_total(v2s,10)==34,"v2 signed frame adds 13 bytes");

  printf("\n%s (%d failures)\n", fails?"TESTS FAILED":"ALL TESTS PASSED", fails);
  return fails;
}
