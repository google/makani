      PARAMETER (NR =  28,
                 NW =  41,
                 NH =  18 )
      REAL RL(NR), WL(NW), HL(NH)
      INTEGER IR1(NH), IR2(NH), IW1(NH), IW2(NH)
      REAL A(NR,NW,NH,2),
          AR(NR,NW,NH,2),
          AW(NR,NW,NH,2),
          AH(NR,NW,NH,2),
         ARW(NR,NW,NH,2),
         ARH(NR,NW,NH,2),
         AWH(NR,NW,NH,2),
        ARWH(NR,NW,NH,2) 
      REAL RL01(90)
      EQUIVALENCE (RL(  1,RL01(1) )
