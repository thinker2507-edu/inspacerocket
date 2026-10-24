#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Local USB text only. Framed commands still use the authenticated wire parser.
struct ConsoleCommand { char name[24]={},arg[48]={}; };
constexpr char consoleUpper(char c){return c>='a' && c<='z'?char(c-'a'+'A'):c;}
constexpr bool consoleEquals(const char* a,const char* b){while(*a && *a==*b){++a;++b;}return *a==*b;}
constexpr void consoleCopy(char* dst,const char* src){while((*dst++=*src++)){} }
constexpr bool parseConsoleCommand(const char* text,ConsoleCommand& out){
  out=ConsoleCommand();
  while(*text==' ')++text;
  size_t n=0;
  while(*text && *text!=' ' && *text!=','){
    char c=consoleUpper(*text++);
    if(n>=sizeof(out.name)-1 || (!(c>='A' && c<='Z') && !(n && ((c>='0' && c<='9') || c=='_'))))return false;
    out.name[n++]=c;
  }
  if(!n)return false;
  while(*text==' ')++text;
  bool comma=*text==',';if(comma)++text;
  while(*text==' ')++text;
  n=0;
  while(*text && *text!=' '){
    char c=consoleUpper(*text++);
    if(n>=sizeof(out.arg)-1 || c<33 || c>126 || c==',' || c=='*' || c=='$')return false;
    out.arg[n++]=c;
  }
  while(*text==' ')++text;
  if(*text || (comma && !n))return false;
  if(consoleEquals(out.name,"TELEM")){
    if(consoleEquals(out.arg,"ON"))consoleCopy(out.arg,"1");
    if(consoleEquals(out.arg,"OFF"))consoleCopy(out.arg,"0");
  }
  if(consoleEquals(out.name,"COUNT") && !*out.arg){consoleCopy(out.name,"STATUS");consoleCopy(out.arg,"COUNTER");}
  return true;
}

struct UsbCommandLine {
  enum Result { NONE, LINE, INVALID };
  char text[100]={};
  uint8_t used=0;
  uint32_t lastByte=0;
  bool dropping=false,skipLF=false;
  constexpr bool expire(uint32_t now){
    if(used && uint32_t(now-lastByte)>1000){used=0;dropping=true;return true;}
    return false;
  }
  constexpr Result feed(int c,uint32_t now){
    if(skipLF){skipLF=false;if(c=='\n')return NONE;}
    if(c=='\r' || c=='\n'){
      skipLF=c=='\r';text[used]=0;
      Result result=dropping?INVALID:used?LINE:NONE;
      used=0;dropping=false;return result;
    }
    lastByte=now;
    if(c<32 || c>126 || used>=sizeof(text)-1)dropping=true;
    else if(!dropping)text[used++]=char(c);
    return NONE;
  }
};
