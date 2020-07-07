      PROGRAM INTAI
      PARAMETER (NH=12, NF=20, NR=100)
      REAL H(NH), A(NR,NF,NH), R(NR,NH)
      REAL M(NH), L(NH)
      REAL F(NF,NH)
      REAL FEN(NR)
      REAL RTN(7)
      INTEGER IH1(7),IH2(7)
C
      CHARACTER*1 ANS
C
      DATA H / 2.2,    2.3,    2.4,    2.5,    2.6,    2.8,    3.0, 
     &         3.4,    4.0,    5.0,    7.0,   10.0 / !,   15.0,   20.0 /
C
      DATA M / 1.4240, 0.3225, 0.1231, 0.0408, -.0031, -.0475, -.0683, 
     &         -.0852, -.0904, -.0868, -.0724, -.0568 / !, -.0415, -.0331 /
C
      DATA L / 0.0637, 0.1876, 0.2902, 0.3756, 0.4471, 0.5569, 0.6330, 
     &         0.7168, 0.7540, 0.7153, 0.5731, 0.4083 / !, 0.2582, 0.1786 /
C
      RTN(1) = 10000.0
      RTN(2) = 5000.0
      RTN(3) = 2000.0
      RTN(4) = 500.0
      RTN(5) = 200.0
      RTN(6) = 100.0
      RTN(7) = 50.0
C
      IH1(1) = 1
      IH2(1) = 3
C
      IH1(2) = 3
      IH2(2) = 5
C
      IH1(3) = 5
      IH2(3) = 7
C
      IH1(4) = 7
      IH2(4) = 9
C
      IH1(5) = 9
      IH2(5) = 11
C
      IH1(6) = 11
      IH2(6) = 13
C
      IH1(7) = 13
      IH2(7) = 14
C
      NPLOT = 7
      NPLOT = 5

      NRANN = 10
C
      ANN = 10.0
      NANN = 5
C
ccc   LMASK = -32640
      LMASK = -30584
ccc   LMASK = -21846
C
      IDEV = 1
      IDEVRP = 2
C
      SIZE = 8.0
      IPSLU = 0
      SCRNFR = 0.85
C
      CALL PLINITIALIZE
C
      PAR = 0.8
      CH = 0.02
C
      DO 5 IH=1, NH
        write(*,*) ih
        CALL NCALC(H(IH),M(IH),L(IH), NR,R(1,IH), NF,F(1,IH),A(1,1,IH))
 5    CONTINUE
C
C
      DO 100 IPLOT=1, NPLOT
        IF(IPLOT.GT.1) CALL PLOT(0.0,0.0,-999)
C
        CALL PLOPEN(SCRNFR,IPSLU,IDEV)
        CALL NEWFACTOR(SIZE)
        CALL PLOT(5.0*CH,5.0*CH,-3)
C
        DELR = RTN(IPLOT)/FLOAT(NRANN)
        RWT = 1.0/RTN(IPLOT)
        CALL XAXIS(0.0,0.0,1.0,RWT*DELR,0.0,DELR,CH,-1)
C
        DA = ANN/FLOAT(NANN)
        AWT = PAR/ANN
        CALL YAXIS(0.0,0.0,PAR,AWT*DA,0.0,DA,CH,-1)
C
        CALL PLGRID(0.0,0.0,NRANN,RWT*DELR,NANN,AWT*DA,LMASK)
C
C
        DO 10 IH=IH1(IPLOT), IH2(IPLOT)
          DO 102 IR=1, NR
            CALL DAMPL(H(IH),R(IR,IH),FEN(IR))
 102      CONTINUE
C
          CALL XYPLOT(NR,R(1,IH),FEN,0.0,RWT,0.0,AWT,2,0.0,0)
C
          DO 105 IR=2, NR
            IFMAX = 1
            AFMAX = 0.0
            DO 1052 IF=1, NF
              IF(A(IR,IF,IH) .GT. AFMAX) THEN
               AFMAX = A(IR,IF,IH)
               IFMAX = IF
              ENDIF
 1052       CONTINUE
            IF(AFMAX.EQ.0.0) GO TO 105
C
ccc            DO 1055 IF=IFMAX, IFMAX
            DO 1055 IF=1, NF
              XPLT1 = RWT*R(IR-1,IH)
              YPLT1 = AWT*A(IR-1,IF,IH)
              XPLT2 = RWT*R(IR,IH)
              YPLT2 = AWT*A(IR,IF,IH)
              IF(YPLT2 .LE. YPLT1) GO TO 1055
              IF(YPLT2 .GT.   PAR) GO TO 1055
              IF(XPLT2 .GT.   1.0) GO TO 10
C
              CALL PLOT(XPLT1,YPLT1,3)
              CALL PLOT(XPLT2,YPLT2,2)
 1055       CONTINUE
 105      CONTINUE
 10     CONTINUE
C
        CALL PLFLUSH
C
        WRITE(*,*) 'Hardcopy ?   N'
        READ(*,8000) ANS
 8000   FORMAT(A)
        IF(INDEX('Yy',ANS).NE.0) CALL REPLOT(IDEVRP)
C
        CALL PLEND
C
 100  CONTINUE
C
      CALL PLCLOSE
      STOP
      END



      SUBROUTINE NCALC(HK,AM,AL, NR,RT, NF,F, A)
C---------------------------------------------------------------
C     Computes N factor for a range of frequencies
C     and Reynolds numbers by integrating growth rates.
C
C     Input:  HK     shape parameter
C             AM     x/Ue dUe/dx
C             AL     theta^2 / nu  dUe/dx
C             NR     number of Rthetas
C             NF     number of frequencies
C
C     Output: RT(.)  Rtheta values
C             F(.)   frequency values
C             A(..)  TS wave amplitudes
C---------------------------------------------------------------
      REAL RT(NR), F(NF), A(NR,NF)
      LOGICAL OK
C
      DW = -2.00/FLOAT(NF-1)
C
      DO 10 IR=1, NR
        DO 105 IF=1, NF
          A(IR,IF) = 0.0
  105   CONTINUE
   10 CONTINUE
C
      HKB = 1.0 / (HK - 1.0)
      RDLC = 2.23 + 1.35*HKB + 0.85*TANH(10.4*HKB - 7.07) - 0.1
      RDC = 10.0**RDLC
      RTC = RDC/HK
C
      WRITE(*,*) 'H Rcr =', HK, RTC
C
      IF(HK.LE.2.21) THEN
       RTN = 3.0*RTC
       DW = -0.20/FLOAT(NF-1)
       WL1 = -1.7
      ELSE IF(HK.LE.2.31) THEN
       RTN = 4.0*RTC
       DW = -0.30/FLOAT(NF-1)
       WL1 = -1.6
      ELSE IF(HK.LE.2.41) THEN
       RTN = 8.0*RTC
       DW = -0.7/FLOAT(NF-1)
       WL1 = -1.5
      ELSE IF(HK.LE.2.51) THEN
       RTN = 12.0*RTC
       DW = -1.20/FLOAT(NF-1)
       WL1 = -1.4
      ELSE IF(HK.LE.2.61) THEN
       RTN = 20.0*RTC
       DW = -1.75/FLOAT(NF-1)
       WL1 = -1.2
      ELSE IF(HK.LE.2.81) THEN
       RTN = 30.0*RTC
       DW = -2.00/FLOAT(NF-1)
       WL1 = -1.0
      ELSE
       RTN = 50.0*RTC
       DW = -2.25/FLOAT(NF-1)
       WL1 = -0.7
      ENDIF
C
ccc      DW = -2.00/FLOAT(NF-1)
C
C
      GEO = (RTN/RTC)**(1.0/FLOAT(NR-1))
      RT(1) = RTC
      DO 20 IR=2, NR
        RT(IR) = RT(IR-1)*GEO
   20 CONTINUE
C
   21 ISTART = 1
C
      REXP = (1.0 - 3.0*AM)/(1.0 + AM)
      AFAC = 0.5*(1.0 + AM) * AL
c
ccc      write(*,*) rexp, afac
C
      IR = ISTART
      UOT1 = RT(IR)**REXP
C
      DO 30 IF=1, NF
        WLOG = WL1 + DW*FLOAT(IF-1)
        F(IF) = 10.0 ** WLOG
   30 CONTINUE
C
      DO 40 IR=ISTART+1, NR
        IRM = IR-1
C
        DRT = RT(IR) - RT(IRM)
        RSP = 0.5*(RT(IR) + RT(IRM))
        HSP = HK
        HSP = AMIN1( HSP , 19.999 )
C
        DO 405 IF=1, NF
          UOT = RSP**REXP
          FSP = F(IF) * (UOT/UOT1)
          CALL OSMAP(RSP,FSP,HSP,
     &                AR,
     &                AR_R, AR_F, AR_H,
     &                ARF_R,ARF_F,ARF_H ,
     &                AI,
     &                AI_R, AI_F, AI_H,
     &                AIF_R,AIF_F,AIF_H , OK)
C
          IF(IR .EQ. ISTART+1) THEN
           IF(AI.LT.0.0) WRITE(*,*) 'Rcrit too high.  H =', HSP
          ENDIF
C
          IF(OK) THEN
           DNDRT = -AI/AFAC
          ELSE
           DNDRT = 0.
          ENDIF
C
          A(IR,IF) = A(IRM,IF) + DNDRT * DRT
          A(IR,IF) = MAX( A(IR,IF) , 0.0 )
  405   CONTINUE
   40 CONTINUE
C
      RETURN
      END


      SUBROUTINE DAMPL(H,RT,AN)
C------------------------------------------------------
C     Returns envelope amplitude for a similar flow.
C
C     Input:  H   shape parameter
C             RT  Rtheta
C
C     Output: AN  n-factor (envelope amplitude)
C------------------------------------------------------
      HMI = H - 1.0
C
      RLCRIT = 2.492/HMI**0.43 + 0.7*(1.0 + TANH(14.0/HMI - 9.24))
      RCRIT = 10.0**RLCRIT
C
      AN = 0.0
      IF(RT .LE. RCRIT) RETURN
C
      DNDR = 0.028*HMI - 0.0345*EXP(-(3.87/HMI - 2.52)**2)
C
      AN = DNDR*(RT - RCRIT)
      RETURN
      END
