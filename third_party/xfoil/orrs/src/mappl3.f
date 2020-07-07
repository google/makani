      PROGRAM MAPGEN
      PARAMETER (NMAX=257,NRX=101,NWX=61,NHX=21)
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      LOGICAL*1 FNAME(32)
      REAL AR(NRX,NWX,NHX), AI(NRX,NWX,NHX), 
     &      X(NRX,NWX,NHX),  Y(NRX,NWX,NHX)
      REAL RT(NRX,NHX),RTL(NRX,NHX)
      REAL WS(NWX,NHX),WSL(NWX,NHX)
      REAL HH(NHX),HHL(NHX)
      INTEGER N(NHX), NRP(NHX), NRW(NHX), NR(NHX),NW(NHX)
C
      CHARACTER*1 ANS
      LOGICAL LABCON, YES
C
      IDEV = 12
      IHARD = 0
      SIZE = 4.5
      CH = 0.020
      CHL = 0.018
C
      CALL READIT(N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
      NH = NHP - 1
      DO 13 IH=1, NHP
        HHL(IH) = HH(IH)
C
        NR(IH) = NRP(IH) - 1
        NW(IH) = NWP(IH) - 1
C
        DO 11 IR=1, NRP(IH)
          RT(IR,IH) = 10.0 ** RTL(IR,IH)
   11   CONTINUE
C
        DO 12 IW=1, NWP(IH)
          WS(IW,IH) = 10.0 ** WSL(IW,IH)
   12   CONTINUE
C
   13 CONTINUE
C
C
      RTLMIN = RTL(1     ,1)
      RTLMAX = RTL(NRP(1),1)
C
      WRLMIN = WSL(1     ,1) - 0.5*RTL(1     ,1)
      WRLMAX = WSL(NWP(1),1) - 0.5*RTL(NRP(1),1)
C
      HHLMIN = HHL(1)
      HHLMAX = HHL(1)
C
      DO 20 IH=1, NHP
        RTLMIN = AMIN1( RTLMIN , RTL(1      ,IH) )
        RTLMAX = AMAX1( RTLMAX , RTL(NRP(IH),IH) )
C
        WRLMIN = AMIN1( WRLMIN ,
     &                  WSL(1      ,IH)-0.5*RLT(1      ,IH))
        WRLMAX = AMAX1( WRLMAX ,
     &                  WSL(NWP(IH),IH)-0.5*RTL(NRP(IH),IH))
C
        HHLMIN = AMIN1( HHLMIN , HHL(IH) )
        HHLMAX = AMAX1( HHLMAX , HHL(IH) )
   20 CONTINUE
C
C
      ARMIN = AR(1,1,1)
      ARMAX = AR(1,1,1)
      AIMIN = AI(1,1,1)
      AIMAX = AI(1,1,1)
      DO 30 IH=1, NHP
        DO 301 IW=1, NWP(IH)
          DO 3010 IR=1, NRP(IH)
            ARMIN = AMIN1(ARMIN,AR(IR,IW,IH))
            ARMAX = AMAX1(ARMAX,AR(IR,IW,IH))
            AIMIN = AMIN1(AIMIN,AI(IR,IW,IH))
            AIMAX = AMAX1(AIMAX,AI(IR,IW,IH))
 3010     CONTINUE
  301   CONTINUE
   30 CONTINUE
C
C
      I1 = INT(RTLMIN+100.001) - 100
      I2 = INT(RTLMAX+100.999) - 100
      J1 = INT(WRLMIN+100.001) - 100
      J2 = INT(WRLMAX+100.999) - 100
      K1 = INT(HHLMIN+100.001) - 100
      K2 = INT(HHLMAX+100.999) - 100
C
      RTLMIN = FLOAT(I1)
      RTLMAX = FLOAT(I2)
      WRLMIN = FLOAT(J1)
      WRLMAX = FLOAT(J2)
      HHLMIN = FLOAT(K1)
      HHLMAX = FLOAT(K2)
C
   90 WRITE(6,*) ' '
      WRITE(6,*) ' 1  W  vs  Rtheta'
      WRITE(6,*) ' 2  H  vs  Rtheta'
      WRITE(6,*) ' 3  W  vs  H'
      WRITE(6,*) ' '
      CALL ASK('Select plot option\',2,IOPT)
C
      GO TO (100,200,300), IOPT
      GO TO 90
C
  100 CALL GETHH(NHX,NHP,HH,IH)
C
      SF = AMIN1( 1.0/(RTLMAX-RTLMIN) , 1.0/(WRLMAX-WRLMIN) )
C
      DO 40 IW=1, NWP(IH)
        DO 401 IR=1, NRP(IH)
          WRL = WSL(IW,IH) - 0.5*RTL(IR,IH)
          X(IR,IW) = (RTL(IR,IH)-RTLMIN) * SF
          Y(IR,IW) = (WRL       -WRLMIN) * SF
  401   CONTINUE
   40 CONTINUE
C
      CALL PLTINI(IHARD,IDEV,SIZE,CH)
      CALL LAXES(I1,I2,J1,J2,SF,CH)
C


      ELSE IF(IOPT.EQ.2) THEN
C

C
      DO 9000 IPASS=1, 2
C
      CALL NEWPEN(3)
      XLAB = 0.5*CH
      YLAB = (FLOAT(J2)-WRLMIN)*SF + 1.5*CH
      CALL SYMBOL(XLAB        ,YLAB-0.4*CH,2.2*CH,'j',0.0,1)
      IF(IPASS.EQ.1)
     &CALL SYMBOL(XLAB+ 1.8*CH,YLAB-0.4*CH,1.2*CH,'I',0.0,1)
      IF(IPASS.EQ.2)
     &CALL SYMBOL(XLAB+ 1.8*CH,YLAB-0.4*CH,1.2*CH,'R',0.0,1)
      CALL SYMBOL(XLAB+ 3.2*CH,YLAB       ,1.8*CH,'0',0.0,1)
      CALL SYMBOL(XLAB+ 3.2*CH,YLAB       ,1.8*CH,'-',0.0,1)
C
      XLAB = (FLOAT(I2)-RTLMIN)*SF - 10.0*1.5*CH
      CALL SYMBOL(XLAB       ,YLAB,1.5*CH,'H = ',0.0,4)
      CALL NUMBER(XLAB+6.0*CH,YLAB,1.5*CH, H    ,0.0,3)
C
      IF(IPASS.EQ.1) WRITE(6,*) 'ai limits:', AIMIN, AIMAX
      IF(IPASS.EQ.2) WRITE(6,*) 'ar limits:', ARMIN, ARMAX
C
  800 CONTINUE
      WRITE(6,*) ' '
      CALL ASK('Enter starting contour level\',3,ALOW)
      CALL ASK('Enter contour level increment (+/-)\',3,DA)
      CALL ASK('Enter contour line thickness (1-5)\',2,LPEN)
      CALL ASK('Add numerical labels to contours ?\',5,LABCON)
C
C
C**** plot and label contours
C
      CALL NEWPEN(LPEN)
C
C---- go over contour levels
      DO 60 IA = 0, 12345
C
C------ set contour level
        ACON = ALOW + DA*FLOAT(IA)
C
C
        IF(IPASS.EQ.1) THEN
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.AIMAX) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.AIMIN)      ) GO TO 61
C
         CALL CON1(NRX,NWX,NRP,NWP,X,Y,AI,ACON,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AI,ACON,1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AI,ACON,1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AI,ACON,1.0,1.0,CHL,3,3)
         ENDIF
        ELSE
C------- skip out if outside limits
         IF((DA.GT.0.0 .AND. ACON.GT.ARMAX) .OR. 
     &      (DA.LT.0.0 .AND. ACON.LT.ARMIN)      ) GO TO 61
C
         CALL CON1(NRX,NWX,NRP,NWP,X,Y,AR,ACON,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AR,ACON,1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AR,ACON,1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NRP,NWP,X,Y,AR,ACON,1.0,1.0,CHL,3,3)
         ENDIF
        ENDIF
   60 CONTINUE
   61 CONTINUE
C
      CALL ASK('Add more contours ?\',5,YES)
      IF(YES) GO TO 800
C
      IF(IPASS.LT.2) CALL PLOT((RTLMAX-RTLMIN)*SF+12.0*CH,0.0,-3)
C
 9000 CONTINUE
C
      CALL PLOT(0.0,0.0,+999)
      STOP
      END

      SUBROUTINE READIT(N,NMAX,ETA,U,S,
     &                  NRP,NWP,NHP,
     &                  RTL,WSL,HH , AR,AI,
     &                  NRX,NWX,NHX)
      DIMENSION N(NHX), NRP(NHX),NWP(NHX)
      DIMENSION ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      DIMENSION AR(NRX,NWX,NHX), AI(NRX,NWX,NHX)
      DIMENSION RTL(NRX), WSL(NWX), HH(NHX)
      LOGICAL*1 FNAME(32)
C
      DO 1000 IH=1, NHX
        CALL ASK('Enter map filename (or <cr> to quit)\',4,FNAME)
        OPEN(9,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=1001)
C
        READ(9) N(IH), HH(IH)
        READ(9) (ETA(I,IH),I=1, N(IH))
        READ(9) (U(I,IH)  ,I=1, N(IH))
        READ(9) (S(I,IH)  ,I=1, N(IH))
        READ(9) NRP(IH), NWP(IH)
        READ(9) (RTL(IR,IH),IR=1,NRP(IH))
        READ(9) (WSL(IW,IH),IW=1,NWP(IH))
C
        DO 10 IW=1, NWP(IH)
          READ(9,END=11) (AR(IR,IW,IH),IR=1,NRP(IH))
          READ(9,END=11) (AI(IR,IW,IH),IR=1,NRP(IH))
   10   CONTINUE
        CLOSE(9)
        GO TO 90
C
   11   CONTINUE
        CLOSE(9)
        IWLAST = IW-1
        WRITE(6,*) 'Map incomplete.'
        WRITE(6,*) 'Last complete frequency index:',IWLAST
C
   90   CONTINUE
        GEO = (ETA(3,IH)-ETA(2,IH)) / (ETA(2,IH)-ETA(1,IH))
C
        WRITE(6,1050) N(IH), HH(IH), ETA(N,IH), GEO
 1050   FORMAT(/' n =', I4,'   H =', F7.3,
     &                     '   Ye =', F7.3,
     &                     '   dYi+1/dYi =',F6.3 /)
 1000 CONTINUE
      IH = NHX + 1
C
 1001 NHP = IH-1
      RETURN
      END


      SUBROUTINE PLTINI(IHARD,IDEV,SIZE,CH)
      CALL PLOTS(0,IHARD,IDEV)
      CALL FACTOR(SIZE)
      CALL PLOT(8.0*CH,8.0*CH,-3)
      RETURN
      END


      SUBROUTINE LAXES(I1,I2,J1,J2,SF,CH)
C
      DO 50 I=I1, I2
        XLIN  = FLOAT(I -I1) * SF
        YLIN1 = FLOAT(J1-J1) * SF
        YLIN2 = FLOAT(J2-J1) * SF
        CALL NEWPEN(1)
        CALL PLOT(XLIN,YLIN1,3)
        CALL PLOT(XLIN,YLIN2,2)
C
        CALL NEWPEN(2)
        RI = FLOAT(I)
        CALL SYMBOL(XLIN-1.0*CH,YLIN1-2.5*CH,1.2*CH,'10',0.0, 2)
        CALL NUMBER(XLIN+1.4*CH,YLIN1-2.0*CH,1.0*CH,RI  ,0.0,-1)
   50 CONTINUE
C
      DO 55 J=J1, J2
        YLIN  = FLOAT(J -J1) * SF
        XLIN1 = FLOAT(I1-I1) * SF
        XLIN2 = FLOAT(I2-I1) * SF
        CALL NEWPEN(1)
        CALL PLOT(XLIN1,YLIN,3)
        CALL PLOT(XLIN2,YLIN,2)
C
        CALL NEWPEN(2)
        RJ = FLOAT(J)
        CALL SYMBOL(XLIN1-4.4*CH,YLIN-0.6*CH,1.2*CH,'10',0.0, 2)
        CALL NUMBER(XLIN1-2.0*CH,YLIN-0.1*CH,1.0*CH,RJ  ,0.0,-1)
   55 CONTINUE
C
      RETURN
      END


      SUBROUTINE RLABEL(I1,I2,J1,SF,CH)
      CALL NEWPEN(2)
      XLAB = (FLOAT((I1+I2)/2) + 0.5 - FLOAT(I1)) * SF - 1.0*CH
      YLAB = (FLOAT( J1      )       - FLOAT(J1)) * SF - 3.5*CH
      CALL SYMBOL(XLAB       ,YLAB       ,1.5*CH,'R',0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB-0.5*CH,1.0*CH,'0',0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB-0.5*CH,1.0*CH,'-',0.0,1)
      RETURN
      END


      SUBROUTINE WLABEL(J1,J2,I1,SF,CH)
      CALL NEWPEN(2)
      XLAB = (FLOAT( I1      )       - FLOAT(I1)) * SF - 6.5*CH
      YLAB = (FLOAT((J1+J2)/2) + 0.5 - FLOAT(J1)) * SF - 0.8*CH
      CALL SYMBOL(XLAB       ,YLAB-0.3*CH,1.5*CH,'h'  ,0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB       ,1.5*CH,'0/U',0.0,3)
      CALL SYMBOL(XLAB+1.5*CH,YLAB       ,1.5*CH,'-'  ,0.0,1)
      RETURN
      END
