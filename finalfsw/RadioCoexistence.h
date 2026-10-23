#pragma once
#include <stdint.h>

// Best-effort shared-channel fairness, NOT synchronized slots or collision proof.
// Applies only to scheduled data/health, never delays a command ACK.
struct RadioCoexistence {
  uint32_t rng=1,deadline=0,peerMs=0;
  bool waiting=false,peerSeen=false;
  constexpr void seed(uint32_t value){rng=value?value:1;}
  constexpr void heardPeer(uint32_t now){peerMs=now;peerSeen=true;waiting=false;}
  constexpr void sent(){waiting=false;}
  constexpr bool ready(uint32_t now){
    if(peerSeen && uint32_t(now-peerMs)<250)return false;
    if(!waiting){
      rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
      deadline=now+100+rng%401;waiting=true;
    }
    return int32_t(now-deadline)>=0;
  }
};
