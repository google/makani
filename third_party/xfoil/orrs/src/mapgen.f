      PROGRAM MAPGEN
      PARAMETER (NMAX=257,NRX=101,NWX=101)
      REAL ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      REAL UTR(NMAX), UTI(NMAX), VTR(NMAX), VTI(NMAX)
      CHARACTER*48 FNAME
      REAL AR(NRX,NWX), AI(NRX,NWX)
      REAL RT(NRX),RTL(NRX), WS(NWX),WSL(NWX)
      LOGICAL CONV(NRX,NWX)
C
      LST = 1
      LRE = 1
C
      WRMAX = 0.25
C
      RESMAX = 0.1
C
C---- default profile parameters
      N = 256
      GEO = 1.02
      ETAE = 14.0
C
      DO 5 IR=1, NRX
        DO 4 IW=1, NWX
          CONV(IR,IW) = .FALSE.
    4   CONTINUE
    5 CONTINUE
C
C---- generate or read in profile
      CALL PFLGET(N,GEO,ETAE,ETA,F,U,S,H)
C
C
      CALL ASKR('Enter lower log10(Rtheta)^',RT1L)
      CALL ASKR('Enter upper log10(Rtheta)^',RT2L)
      CALL ASKI('Enter number of log10(Rtheta )intervals^',NR)
C
      CALL ASKR('Enter lower log10(Wr*sqrt(Rtheta))^',WS1L)
      CALL ASKR('Enter upper log10(Wr*sqrt(Rtheta))^',WS2L)
      CALL ASKI('Enter number of log10(Wr) intervals^',NW)
C
      NRP = NR + 1
      NWP = NW + 1
C
      IF(NRP.GT.NRX) STOP 'Array overflow'
      IF(NWP.GT.NWX) STOP 'Array overflow'
C
      RT1 = 10.0 ** RT1L
      RT2 = 10.0 ** RT2L
      DO 10 IR=1, NRP
        RTL(IR) = RT1L + (RT2L-RT1L)*FLOAT(IR-1)/FLOAT(NR)
        RT(IR) = 10.0 ** RTL(IR)
   10 CONTINUE
C
      WS1 = 10.0 ** WS1L
      WS2 = 10.0 ** WS2L
      DO 15 IW=1, NWP
        WSL(IW) = WS1L + (WS2L-WS1L)*FLOAT(IW-1)/FLOAT(NW)
        WS(IW) = 10.0 ** WSL(IW)
   15 CONTINUE
C
C
      CALL ASKR('Enter initial  ar  for lower Rtheta, upper Wr^',AR0)
      CALL ASKR('Enter initial  ai  for lower Rtheta, upper Wr^',AI0)
C
C
      CALL ASKS('Enter map output filename^',FNAME)
      OPEN(19,FILE=FNAME,STATUS='NEW',FORM='UNFORMATTED')
      WRITE(19) N, H
      WRITE(19) (ETA(I),I=1, N)
      WRITE(19) (U(I)  ,I=1, N)
      WRITE(19) (S(I)  ,I=1, N)
      WRITE(19) NRP, NWP
      WRITE(19) (RTL(IR),IR=1,NRP)
      WRITE(19) (WSL(IW),IW=1,NWP)
C
      IR1 = NRP
      IR2 = 1
      IRD = -1
C
      DO 100 IW=1, NWP
        WRITE(6,2010)
 2010   FORMAT(/1X,'--------------------')
        DO 90 IR=IR1, IR2, IRD
C
          WR = WS(IW)/SQRT(RT(IR))
C
          WRITE(6,2020) IR,IW, RT(IR), WR
 2020     FORMAT(/1X,2I4,'    Rth =', E12.4, '    Wr =', E12.4)
C
          WR0 = WR
          WI0 = 0.0
C
C-------- set initial wavenumber guess
          IRM1 = IR -   IRINCR
          IRM2 = IR - 2*IRINCR
          IRM3 = IR - 3*IRINCR
C
          IWM1 = IW -   IWINCR
          IWM2 = IW - 2*IWINCR
          IWM3 = IW - 3*IWINCR
C
          IF(IRM2.GE.1 .AND. IRM2.LE.NRP  .AND.
     &       IWM1.GE.1 .AND. IWM1.LE.NWP        ) THEN
            AR0 =                 2.0*AR(IRM1,IW  ) - AR(IRM2,IW  )
     &          + AR(IR  ,IWM1) - 2.0*AR(IRM1,IWM1) + AR(IRM2,IWM1)
            AI0 =                 2.0*AI(IRM1,IW  ) - AI(IRM2,IW  )
     &          + AI(IR  ,IWM1) - 2.0*AI(IRM1,IWM1) + AI(IRM2,IWM1)
          ELSE IF(IRM1.GE.1 .AND. IRM1.LE.NRP  .AND.
     &            IWM2.GE.1 .AND. IWM2.LE.NWP        ) THEN
            AR0 =                         AR(IRM1,IW  )
     &          + 2.0*AR(IR  ,IWM1) - 2.0*AR(IRM1,IWM1)
     &          -     AR(IR  ,IWM2) +     AR(IRM1,IWM2)
            AI0 =                         AI(IRM1,IW  )
     &          + 2.0*AI(IR  ,IWM1) - 2.0*AI(IRM1,IWM1)
     &          -     AI(IR  ,IWM2) +     AI(IRM1,IWM2)
          ELSE IF(IRM1.GE.1 .AND. IRM1.LE.NRP  .AND.
     &            IWM1.GE.1 .AND. IWM1.LE.NWP        ) THEN
            AR0 =                 AR(IRM1,IW  )
     &          + AR(IR  ,IWM1) - AR(IRM1,IWM1)
            AI0 =                 AI(IRM1,IW  )
     &          + AI(IR  ,IWM1) - AI(IRM1,IWM1)
          ELSE IF(IRM2.GE.1 .AND. IRM2.LE.NRP) THEN
            AR0 = 2.0*AR(IRM1,IW) - AR(IRM2,IW)
            AI0 = 2.0*AI(IRM1,IW) - AI(IRM2,IW)
          ELSE IF(IWM2.GE.1 .AND. IWM2.LE.NWP) THEN
            AR0 = 2.0*AR(IR,IWM1) - AR(IR,IWM2)
            AI0 = 2.0*AI(IR,IWM1) - AI(IR,IWM2)
          ELSE IF(IRM1.GE.1 .AND. IRM1.LE.NRP) THEN
            AR0 = AR(IRM1,IW)
            AI0 = AI(IRM1,IW)
          ELSE IF(IWM1.GE.1 .AND. IWM1.LE.NWP) THEN
            AR0 = AR(IR,IWM1)
            AI0 = AI(IR,IWM1)
CCC          ELSE
CCC            STOP 'Cannot start in corner and go in'
          ENDIF
c
          AR(IR,IW) = AR0
          AI(IR,IW) = AI0
C
C-------- don't bother with absurdly high frequency
          IF(WR .GE. WRMAX) THEN
           DELMAX = 0.0
           GO TO 89
          ENDIF
C
          ITMAX = 10
          CALL ORRS(LST,LRE,N,ETA,U,S, RT(IR), ITMAX,
     &              AR0,AI0, WR0,WI0, UTR,UTI,VTR,VTI,DELMAX)
C
   89     IF(DELMAX.LT.RESMAX) CONV(IR,IW) = .TRUE.
C
          AR(IR,IW) = AR0
          AI(IR,IW) = AI0
C
   90   CONTINUE
C
        WRITE(19) (AR(IR,IW),IR=1,NRP)
        WRITE(19) (AI(IR,IW),IR=1,NRP)
C
  100 CONTINUE
C
      CLOSE(19)
C
      STOP
      END


      SUBROUTINE PFLGET(N,GEO,ETAE,ETA,F,U,S,H)
      DIMENSION ETA(N),F(N),U(N),S(N)
      CHARACTER*48 FNAME
C
C---- eta coordinate normalized with momentum thickness
      INORM = 3
C
      WRITE(6,*) ' '
      WRITE(6,*) '  1   Falkner-Skan parameter m = x/U dU/dx'
      WRITE(6,*) '  2   Falkner-Skan parameter beta = 2m/(m+1)'
      WRITE(6,*) '  3   Falkner-Skan shape parameter H'
      WRITE(6,*) '  4   General profile input file'
      WRITE(6,*) ' '
      CALL ASKI('Select profile option^',IOPT)
C
      IF(IOPT.NE.4) THEN
       CALL ASKI('Enter number of BL points^',N)
       CALL ASKR('Enter geometric stretching factor^',GEO)
       CALL ASKR('Enter edge eta value^',ETAE)
      ENDIF
C
C
      IF(IOPT.EQ.1) THEN
C
       CALL ASKR('Enter m^',BU)
       CALL FS(INORM,1,BU,H,N,ETAE,GEO,ETA,F,U,S)
C
      ELSE IF(IOPT.EQ.2) THEN
C
       CALL ASKR('Enter beta^',BETA)
       BU = BETA/(2.0-BETA)
       CALL FS(INORM,1,BU,H,N,ETAE,GEO,ETA,F,U,S)
C
      ELSE IF(IOPT.EQ.3) THEN
C
       CALL ASKR('Enter H^',H)
       CALL FS(INORM,2,BU,H,N,ETAE,GEO,ETA,F,U,S)
C
      ELSE
C
       CALL ASKS('Enter profile filename^',FNAME)
       OPEN(1,FILE=FNAME,STATUS='OLD')
       READ(1,*) N, H
       DO 5 I=1, N
         READ(1,*) ETA(I), U(I), S(I)
    5  CONTINUE
       CLOSE(1)
C
       GEO = (ETA(3)-ETA(2)) / (ETA(2)-ETA(1))
      ENDIF
C
      WRITE(6,1050) N, H, ETA(N), GEO
 1050 FORMAT(/' n =', I4,'   H =', F7.3,
     &                   '   Ye =', F7.3,
     &                   '   dYi+1/dYi =',F6.3 /)
C
      RETURN
      END
