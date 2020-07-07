
C---- variable indices
      PARAMETER (
     &  IVALFA =  1,
     &  IVBETA =  2,
     &  IVROTX =  3,
     &  IVROTY =  4,
     &  IVROTZ =  5,
     &  IVTOT  =  5 )

C---- constraint indices
      PARAMETER (
     &  ICALFA =  1,
     &  ICBETA =  2,
     &  ICROTX =  3,
     &  ICROTY =  4,
     &  ICROTZ =  5,
     &  ICCL   =  6,
     &  ICCY   =  7,
     &  ICMOMX =  8,
     &  ICMOMY =  9,
     &  ICMOMZ = 10,
     &  ICTOT  = 10 )
C
C---- run-case dimensional parameter indices
      PARAMETER (
     &  IPALFA  =  1,
     &  IPBETA  =  2,
     &  IPROTX  =  3,
     &  IPROTY  =  4,
     &  IPROTZ  =  5,
     &  IPCL    =  6,
     &  IPCD0   =  7,
     &  IPPHI   =  8,
     &  IPTHE   =  9,
     &  IPPSI   = 10,
     &  IPMACH  = 11,
     &  IPVEE   = 12,
     &  IPRHO   = 13,
     &  IPGEE   = 14,
     &  IPRAD   = 15,
     &  IPFAC   = 16,
     &  IPXCG   = 17,
     &  IPYCG   = 18,
     &  IPZCG   = 19,
     &  IPMASS  = 20,
     &  IPIXX   = 21,
     &  IPIYY   = 22,
     &  IPIZZ   = 23,
     &  IPIXY   = 24,
     &  IPIYZ   = 25,
     &  IPIZX   = 26,
     &  IPCLA   = 27,
     &  IPCLU   = 28,
     &  IPCMA   = 29,
     &  IPCMU   = 30,
     &  IPTOT   = 30)

C---- eigenmode variable indices
      PARAMETER(
     &  JEU   =  1,
     &  JEW   =  2,
     &  JEQ   =  3,
     &  JETH  =  4,
     &  JEV   =  5,
     &  JEP   =  6,
     &  JER   =  7,
     &  JEPH  =  8,
     &  JEX   =  9,
     &  JEY   = 10,
     &  JEZ   = 11,
     &  JEPS  = 12,
     &  JETOT = 12 )

C---- time-trace parameter indices
      PARAMETER (KPBANK =  1,
     &           KPELEV =  2,
     &           KPHEAD =  3,
     &           KPVINF =  4,
     &           KPALFA =  5,
     &           KPBETA =  6,
     &           KPCLIF =  7,
     &           KPCDRG =  8,
     &           KPCDIN =  9,
     &           KPCMOM = 10,
     &           KPLIFT = 11,
     &           KPDRAG = 12,
     &           KPMACH = 13,
     &           KPALTK = 14,
     &           KPDENS = 15,
     &           KPVSOU = 16,
     &           KPLOAD = 17 )
      PARAMETER (KPTOT  = 17 )
      PARAMETER (KPVAC =  1,
     &           KPRAC =  2,
     &           KPVEL =  3,
     &           KPROT =  4,  
     &           KPPOS =  5,
     &           KPFOR =  6,
     &           KPMOM =  7 )
      PARAMETER (KPVTOT = 7 )
