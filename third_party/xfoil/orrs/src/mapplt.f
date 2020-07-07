      PROGRAM MAPPLT
      PARAMETER (NMAX=257,NRX=101,NWX=91,NHX=21)
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      LOGICAL*1 FNAME(32)
      REAL AR(NRX,NWX,NHX), AI(NRX,NWX,NHX)
      REAL X(NRX,NWX),  Y(NRX,NWX)
      REAL RT(NRX,NHX),RTL(NRX,NHX)
      REAL WS(NWX,NHX),WSL(NWX,NHX)
      REAL HH(NHX),HHL(NHX)
      INTEGER N(NHX), NRP(NHX), NWP(NHX), NR(NHX),NW(NHX)
C
      CHARACTER*1 ANS
      LOGICAL LABCON, YES
C
      IDEV = 12
      IHARD = 0
      SIZE = 4.0
      CH = 0.020
      CHL = 0.018
C
C---- log-log Rtheta-W plot exponent limits
      I1 = 0
      I2 = 6
      J1 = -5
      J2 = 1
C
      CALL PLOTS(0,IHARD,IDEV)
      CALL FACTOR(SIZE)
      CALL PLOT(8.0*CH,8.0*CH,-3)
C
      CALL READIT(N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
      NH = NHP - 1
      DO 15 IH=1, NHP
        HHL(IH) = HH(IH)
C
        NR(IH) = NRP(IH) - 1
        NW(IH) = NWP(IH) - 1
C
        DO 13 IR=1, NRP(IH)
          RT(IR,IH) = 10.0 ** RTL(IR,IH)
   13   CONTINUE
C
        DO 14 IW=1, NWP(IH)
          WS(IW,IH) = 10.0 ** WSL(IW,IH)
   14   CONTINUE
C
   15 CONTINUE
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
      RTLMIN = RTL(1     ,1)
      RTLMAX = RTL(NRP(1),1)
      WRLMIN = WSL(1     ,1) - 0.5*RTL(1     ,1)
      WRLMAX = WSL(NWP(1),1) - 0.5*RTL(NRP(1),1)
      HHLMIN = HHL(1)
      HHLMAX = HHL(1)
      DO 20 IH=1, NHP
        RTLMIN = AMIN1( RTLMIN , RTL(1      ,IH) )
        RTLMAX = AMAX1( RTLMAX , RTL(NRP(IH),IH) )
        WRLMIN = AMIN1( WRLMIN ,
     &                  WSL(1      ,IH)-0.5*RTL(1      ,IH))
        WRLMAX = AMAX1( WRLMAX ,
     &                  WSL(NWP(IH),IH)-0.5*RTL(NRP(IH),IH))
        HHLMIN = AMIN1( HHLMIN , HHL(IH) )
        HHLMAX = AMAX1( HHLMAX , HHL(IH) )
   20 CONTINUE
C
C
      RTLMIN = FLOAT(I1)
      RTLMAX = FLOAT(I2)
      WRLMIN = FLOAT(J1)
      WRLMAX = FLOAT(J2)
C
      SF = AMIN1( 1.0/(RTLMAX-RTLMIN) , 1.0/(WRLMAX-WRLMIN) )
C
C
      DO 2000 IPASS=1, 2
C
      WRITE(6,*) ' '
      IF(IPASS.EQ.1) WRITE(6,*) 'ai limits:', AIMIN, AIMAX
      IF(IPASS.EQ.2) WRITE(6,*) 'ar limits:', ARMIN, ARMAX
C
      WRITE(6,*) ' '
      WRITE(6,*) 'Enter contour level'
      READ (5,*) ACON
      WRITE(6,*) 'Enter contour line thickness (1-5)'
      READ (5,*) LPEN
      WRITE(6,*) 'Add H labels to contours ?  N'
      READ (5,9900) ANS
 9900 FORMAT(A1)
      LABCON = ANS.EQ.'Y'
C
c      CALL ASK('Enter contour level\',3,ACON)
c      CALL ASK('Enter contour line thickness (1-5)\',2,LPEN)
c      CALL ASK('Add H labels to contours ?\',5,LABCON)
C
      DO 50 I=I1, I2
        XLIN  = (FLOAT(I) -RTLMIN) * SF
        YLIN1 = (FLOAT(J1)-WRLMIN) * SF
        YLIN2 = (FLOAT(J2)-WRLMIN) * SF
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
        YLIN  = (FLOAT(J) -WRLMIN) * SF
        XLIN1 = (FLOAT(I1)-RTLMIN) * SF
        XLIN2 = (FLOAT(I2)-RTLMIN) * SF
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
      CALL NEWPEN(2)
      XLAB = (FLOAT((I1+I2)/2) + 0.5 - RTLMIN) * SF - 1.5*CH
      YLAB = (FLOAT( J1      )       - WRLMIN) * SF - 3.5*CH
      CALL SYMBOL(XLAB       ,YLAB       ,1.5*CH,'R',0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB-0.5*CH,1.0*CH,'0',0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB-0.5*CH,1.0*CH,'-',0.0,1)
C
      CALL NEWPEN(2)
      XLAB = (FLOAT( I1      )       - RTLMIN) * SF - 6.5*CH
      YLAB = (FLOAT((J1+J2)/2) + 0.5 - WRLMIN) * SF - 0.8*CH
      CALL SYMBOL(XLAB       ,YLAB-0.3*CH,1.5*CH,'h'  ,0.0,1)
      CALL SYMBOL(XLAB+1.5*CH,YLAB       ,1.5*CH,'0/U',0.0,3)
      CALL SYMBOL(XLAB+1.5*CH,YLAB       ,1.5*CH,'-'  ,0.0,1)
C
      CALL NEWPEN(3)
      XLAB = 0.5*CH
      YLAB = (FLOAT(J2)-WRLMIN)*SF + 1.5*CH
      CALL SYMBOL(XLAB       ,YLAB,1.8*CH,'H ',0.0,2)
      CALL SYMBOL(XLAB+3.6*CH,YLAB,1.4*CH,'CONTOURS',0.0,8)
C
      XLAB = (FLOAT(I2)-RTLMIN)*SF - 10.0*1.5*CH
      CALL SYMBOL(XLAB        ,YLAB-0.4*CH,1.9*CH,'j',0.0,1)
      IF(IPASS.EQ.1)
     &CALL SYMBOL(XLAB+ 1.5*CH,YLAB-0.4*CH,1.2*CH,'I',0.0,1)
      IF(IPASS.EQ.2)
     &CALL SYMBOL(XLAB+ 1.5*CH,YLAB-0.4*CH,1.2*CH,'R',0.0,1)
      CALL SYMBOL(XLAB+ 2.9*CH,YLAB       ,1.5*CH,'0',0.0,1)
      CALL SYMBOL(XLAB+ 2.9*CH,YLAB       ,1.5*CH,'-',0.0,1)
      CALL SYMBOL(XLAB+ 4.4*CH,YLAB       ,1.5*CH,' = ',0.0,3)
      CALL NUMBER(XLAB+ 8.9*CH,YLAB       ,1.5*CH,ACON ,0.0,3)
C
  800 CONTINUE
C
C**** plot and label contours
C
      CALL NEWPEN(LPEN)
C
C---- go over shape parameters
      DO 80 IH = 1, NHP
C
        DO 40 IW=1, NWP(IH)
          DO 401 IR=1, NRP(IH)
            WRL = WSL(IW,IH) - 0.5*RTL(IR,IH)
            X(IR,IW) = (RTL(IR,IH)-RTLMIN) * SF
            Y(IR,IW) = (WRL       -WRLMIN) * SF
  401     CONTINUE
   40   CONTINUE
C
        IF(IPASS.EQ.1) THEN
         CALL CON1(NRX,NWX,NRP(IH),NWP(IH),X,Y,AI(1,1,IH),ACON,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AI(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AI(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AI(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,3)
         ENDIF
        ELSE
         CALL CON1(NRX,NWX,NRP(IH),NWP(IH),X,Y,AR(1,1,IH),ACON,1.0,1.0)
C
C------- draw label contours on bottom, right, and top edges
         IF(LABCON) THEN
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AR(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,1)
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AR(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,2)
          CALL CONLAB(NRX,NWX,NRP(IH),NWP(IH),X,Y,AR(1,1,IH),HH(IH),
     &                1.0,1.0,CHL,3,3)
         ENDIF
        ENDIF
   80 CONTINUE
   81 CONTINUE
C
      IF(IPASS.LT.2) CALL PLOT((RTLMAX-RTLMIN)*SF+12.0*CH,0.0,-3)
C
 2000 CONTINUE
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
      DIMENSION AR(NRX,NWX,NHX),AI(NRX,NWX,NHX)
      DIMENSION RTL(NRX,NHX), WSL(NWX,NHX), HH(NHX)
      LOGICAL*1 FNAME(32)
C
      OPEN(10,FILE='AIMAPS.DAT',STATUS='OLD')
C
      DO 1000 IH=1, NHX
C
        READ(10,5000,END=1001) FNAME
 5000   FORMAT(32A1)
        FNAME(32) = 0
C
        OPEN(9,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=1001)
        READ(9,ERR=1001) N(IH), HH(IH)
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
        GO TO 30
C
   11   CONTINUE
        CLOSE(9)
        IWLAST = IW-1
        WRITE(6,*) 'Map incomplete.'
        WRITE(6,*) 'Last complete frequency index:',IWLAST
C
   30   CONTINUE
        GEO = (ETA(3,IH)-ETA(2,IH)) / (ETA(2,IH)-ETA(1,IH))
C
        WRITE(6,1050) N(IH), HH(IH), ETA(N(IH),IH), GEO
 1050   FORMAT(/' n =', I4,'   H =', F7.3,
     &                     '   Ye =', F7.3,
     &                     '   dYi+1/dYi =',F6.3 /)
 1000 CONTINUE
      IH = NHX + 1
C
 1001 NHP = IH-1
      CLOSE(10)
      CLOSE(9)
C
      DO 40 IH=1, NHP
        IF(RTL(1,IH) .GT. RTL(NRP(IH),IH)) THEN
C
         DO 405 IR=1, NRP(IH)/2
           IRBACK = NRP(IH)-IR+1
C
           RTEMP = RTL(IR,IH)
           RTL(IR,IH) = RTL(IRBACK,IH)
           RTL(IRBACK,IH) = RTEMP
C
           DO 4055 IW=1, NWP(IH)
             AITEMP = AI(IR,IW,IH)
             AI(IR,IW,IH) = AI(IRBACK,IW,IH)
             AI(IRBACK,IW,IH) = AITEMP
 4055      CONTINUE
  405    CONTINUE
C
        ENDIF
C
        IF(WSL(1,IH) .GT. WSL(NWP(IH),IH)) THEN
C
         DO 407 IW=1, NWP(IH)/2
           IWBACK = NWP(IH)-IW+1
C
           WTEMP = WSL(IW,IH)
           WSL(IW,IH) = WSL(IWBACK,IH)
           WSL(IWBACK,IH) = WTEMP
C
           DO 4075 IR=1, NRP(IH)
             AITEMP = AI(IR,IW,IH)
             AI(IR,IW,IH) = AI(IR,IWBACK,IH)
             AI(IR,IWBACK,IH) = AITEMP
 4075      CONTINUE
  407    CONTINUE
C
        ENDIF
   40 CONTINUE
C
      RETURN
      END


