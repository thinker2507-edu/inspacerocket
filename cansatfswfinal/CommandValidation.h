#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

constexpr bool decimalU32(const char* s) {
  if (!*s) return false;
  uint32_t n = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    uint32_t digit = (uint32_t)(*s - '0');
    if (n > (UINT32_MAX - digit) / 10U) return false;
    n = n * 10U + digit;
  }
  return true;
}

constexpr int commandHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Canonical fleet address: decimal team followed by :R (rocket) or :C (CanSat).
// Unaddressed legacy commands deliberately fail closed.
constexpr bool validVehicleAddress(const char* s, size_t n) {
  if(n<3 || s[n-2]!=':' || (s[n-1]!='R' && s[n-1]!='C'))return false;
  uint32_t team=0;
  for(size_t i=0;i<n-2;++i){
    if(s[i]<'0' || s[i]>'9' || (i==0 && n>3 && s[i]=='0'))return false;
    uint32_t digit=s[i]-'0';
    if(team>(UINT32_MAX-digit)/10U)return false;
    team=team*10U+digit;
  }
  return true;
}
constexpr bool vehicleAddressMatches(const char* s,size_t n,uint32_t team,char vehicle) {
  if(!validVehicleAddress(s,n) || s[n-1]!=vehicle)return false;
  uint32_t parsed=0;
  for(size_t i=0;i<n-2;++i)parsed=parsed*10U+(s[i]-'0');
  return parsed==team;
}

// Validate the entire radio payload before tokenization or any side effect.
constexpr bool validCommandFrame(const char* s, size_t len) {
  if (len < 9 || len > 99) return false;
  const char* prefix = "$CMD,";
  for (size_t i = 0; i < 5; ++i) if (s[i] != prefix[i]) return false;
  size_t end=5;
  while(end<len && s[end]!=',')++end;
  if(!validVehicleAddress(s+5,end-5))return false;
  size_t star = len - 3;
  if (s[star] != '*' || commandHex(s[star + 1]) < 0 || commandHex(s[star + 2]) < 0) return false;
  uint8_t cs = 0;
  unsigned commas = 0;
  for (size_t i = 1; i < star; ++i) {
    unsigned char c = (unsigned char)s[i];
    if (c < 33 || c > 126 || c == '$' || c == '*') return false;
    if (c == ',') {
      if (s[i - 1] == ',' || i + 1 == star) return false;
      ++commas;
    }
    cs ^= c;
  }
  return (commas == 4 || commas == 5) && cs == (commandHex(s[star + 1]) * 16 + commandHex(s[star + 2]));
}

constexpr bool commandAddressMatches(const char* s,size_t n,uint32_t team,char vehicle) {
  if(!validCommandFrame(s,n))return false;
  size_t end=5;while(end<n && s[end]!=',')++end;
  return vehicleAddressMatches(s+5,end-5,team,vehicle);
}

// Checksum-checked fleet identity, used only to defer routine TX, never to act.
constexpr char fleetDownlinkVehicle(const char* s,size_t n,uint32_t team) {
  if(n<7 || s[n-3]!='*' || commandHex(s[n-2])<0 || commandHex(s[n-1])<0)return 0;
  size_t start=0;
  if(s[0]=='$'){
    if(n<12 || (s[1]!='A' && s[1]!='S'))return 0;
    if(!((s[1]=='A' && s[2]=='C' && s[3]=='K') ||
         (s[1]=='S' && s[2]=='Y' && s[3]=='S')) || s[4]!=',')return 0;
    start=5;
  }
  uint8_t cs=0;for(size_t i=s[0]=='$'?1:0;i<n-3;++i)cs^=uint8_t(s[i]);
  if(cs!=commandHex(s[n-2])*16+commandHex(s[n-1]))return 0;
  size_t end=start;while(end<n-3 && s[end]!=',')++end;
  if(end>=n-3 || !validVehicleAddress(s+start,end-start))return 0;
  char vehicle=s[end-1];
  return vehicleAddressMatches(s+start,end-start,team,vehicle)?vehicle:0;
}

// Bounded in-RAM history prevents delayed duplicates from repeating side effects.
// The persistent last-command outcome remains handled by the flight journal.
struct CommandHistory {
  uint32_t entries[16]={};uint8_t used=0,next=0;
  constexpr bool contains(uint32_t seq)const{
    for(uint8_t i=0;i<used;++i)if(entries[i]==seq)return true;
    return false;
  }
  constexpr void add(uint32_t seq){entries[next]=seq;next=(next+1)%16;if(used<16)++used;}
};
