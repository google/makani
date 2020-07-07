      PROGRAM MAPMOD
      PARAMETER (NMAX=257,NRX=101,NWX=101)
      REAL ETA(NMAX), F(NMAX), U(NMAX), S(NMAX)
      REAL UTR(NMAX), UTI(NMAX), VTR(NMAX), VTI(NMAX)
      CHARACTER*48 FNAME
      REAL AR(NRX,NWX), AI(NRX,NWX)
      REAL RT(NRX),RTL(NRX), WS(NWX),WSL(NWX)
C
      LST = 1
      LRE = 1
C
      WRMAX = 0.15
      RESMAX = 0.01
C
      NH = 1
      NHX = 1
      CALL ASKS('Enter map filename^',FNAME)
      CALL READOS(FNAME,1,
     &            N,H,ETA,U,S,
     &            NRP,NWP,NH,
     &            RTL,WSL,HDUM,
     &            AR,AI,
     &            NRX,NWX,NHX)
      NR = NRP - 1
      NW = NWP - 1
C
      DO 10 IR=1, NRP
        RT(IR) = 10.0 ** RTL(IR)
   10 CONTINUE
C
      DO 15 IW=1, NWP
        WS(IW) = 10.0 ** WSL(IW)
   15 CONTINUE
C
C
      WRITE(*,1200) RTL(1), RTL(NRP), NR, WSL(1), WSL(NWP), NW
 1200 FORMAT(/' log(Rth) :  low =', F7.4,'   high =', F7.4,'   NR =',I3
     &       /' log(W*sR):  low =', F7.4,'   high =', F7.4,'   NW =',I3)
C
      WRITE(*,*) ' '
      WRITE(*,*) '1  Add/replace scaled frequencies'
      WRITE(*,*) '2  Add/replace Reynolds numbers'
      WRITE(*,*) ' '
      CALL ASKI('Select option^',IOPT)
      WRITE(*,*) ' '
C
      IF(IOPT.EQ.1) THEN
C
C----- get starting and final frequency indices
       CALL GETFR(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IW1,IW2)
       IWINCR = ISIGN( 1 , (IW2-IW1) )
C
       CALL ASKI('Enter Re index (+/- dir) to start at^',IR1S)
       IR1 = IABS(IR1S)
       IF(IR1S.GT.0) IR2 = NRP
       IF(IR1S.LT.0) IR2 = 1
       IRINCR = ISIGN( 1 , IR1S )
C
       IF(IW2 .GT. NWP) THEN
C
C------ 2nd index past current max --- set new max number of frequencies
        NWP = IW2
        IF(NWP .GT. NWX) STOP 'Array overflow'
C
       ELSE IF(IW2 .LT. 1) THEN
C
C------ 2nd index less than 1 --- move arrays to make space...
        NWMOV = 1 - IW2
        DO 20 IW=NWP, 1, -1
          WSL(IW+NWMOV) = WSL(IW)
          WSL(IW) = 0.0
          WS(IW+NWMOV) = WS(IW)
          WS(IW) = 0.0
          DO 205 IR=1, NRP
            AR(IR,IW+NWMOV) = AR(IR,IW)
            AI(IR,IW+NWMOV) = AI(IR,IW)
            AR(IR,IW) = 0.0
            AI(IR,IW) = 0.0
  205     CONTINUE
   20   CONTINUE
        IW1 = IW1 + NWMOV
        IW2 = IW2 + NWMOV
        NWP = NWP + NWMOV
        IF(NWP .GT. NWX) STOP 'Array overflow'
       ENDIF
C
C----- set new frequencies
       DWSL = WSL(IW1-IWINCR) - WSL(IW1-2*IWINCR)
       DO 25 IW=IW1, IW2, IWINCR
         WSL(IW) = WSL(IW-IWINCR) + DWSL
         WS(IW) = 10.0 ** WSL(IW)
   25  CONTINUE
C
      ELSE
C
       CALL GETRE(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IR1,IR2)
       IRINCR = ISIGN( 1 , (IR2-IR1) )
C
       CALL ASKI('Enter W index (+/- dir) to start at^',IW1S)
       IW1 = IABS(IW1S)
       IF(IW1S.GT.0) IW2 = NWP
       IF(IW1S.LT.0) IW2 = 1
       IWINCR = ISIGN( 1 , IW1S )
C
       IF(IR2 .GT. NRP) THEN
        NRP = IR2
        IF(NRP .GT. NRX) STOP 'Array overflow'
       ELSE IF(IR2 .LT. 1) THEN
        NRMOV = 1 - IR2
        DO 30 IR=NRP, 1, -1
          RTL(IR+NRMOV) = RTL(IR)
          RTL(IR) = 0.0
          RT(IR+NRMOV) = RT(IR)
          RT(IR) = 0.0
          DO 305 IW=1, NWP
            AR(IR+NRMOV,IW) = AR(IR,IW)
            AI(IR+NRMOV,IW) = AI(IR,IW)
            AR(IR,IW) = 0.0
            AI(IR,IW) = 0.0
  305     CONTINUE
   30   CONTINUE
        IR1 = IR1 + NRMOV
        IR2 = IR2 + NRMOV
        NRP = NRP + NRMOV
        IF(NRP .GT. NRX) STOP 'Array overflow'
       ENDIF
C
       DRTL = RTL(IR1-IRINCR) - RTL(IR1-2*IRINCR)
       DO 35 IR=IR1, IR2, IRINCR
         RTL(IR) = RTL(IR-IRINCR) + DRTL
         RT(IR) = 10.0 ** RTL(IR)
   35  CONTINUE
C
      ENDIF
C
C---------------------
C
      CALL ASKS('Enter map output filename^',FNAME)
      OPEN(19,FILE=FNAME,STATUS='NEW',FORM='UNFORMATTED')
      WRITE(19) N, H
      WRITE(19) (ETA(I),I=1, N)
      WRITE(19) (U(I)  ,I=1, N)
      WRITE(19) (S(I)  ,I=1, N)
      WRITE(19)  NRP, NWP
      WRITE(19) (RTL(IR),IR=1,NRP)
      WRITE(19) (WSL(IW),IW=1,NWP)
C
      DO 80 IW=IW1, IW2, IWINCR
C
        WRITE(*,2010)
 2010   FORMAT(/1X,'--------------------')
        DO 810 IR=IR1, IR2, IRINCR
C
          WR = WS(IW)/SQRT(RT(IR))
C
          WRITE(*,2020) IW,IR, RT(IR), WR
 2020     FORMAT(/1X,2I4,'    Rth =', E12.4, '    Wr =', E12.4)
C
          WR0 = WR
          WI0 = 0.0
C
C
          IRM1 = IR -   IRINCR
          IRM2 = IR - 2*IRINCR
          IRM3 = IR - 3*IRINCR
C
          IWM1 = IW -   IWINCR
          IWM2 = IW - 2*IWINCR
          IWM3 = IW - 3*IWINCR
C
ccc            AR0 = 2.0*AR(IR,IWM1) - AR(IR,IWM2)
ccc            AI0 = 2.0*AI(IR,IWM1) - AI(IR,IWM2)

          IF(IRM2.GE.1 .AND. IRM2.LE.NRP  .AND.
     &       IWM1.GE.1 .AND. IWM1.LE.NWP        ) THEN
            AR0 =                 2.0*AR(IRM1,IW  ) - AR(IRM2,IW  )
     &          + AR(IR  ,IWM1) - 2.0*AR(IRM1,IWM1) + AR(IRM2,IWM1)
            AI0 =                 2.0*AI(IRM1,IW  ) - AI(IRM2,IW  )
     &          + AI(IR  ,IWM1) - 2.0*AI(IRM1,IWM1) + AI(IRM2,IWM1)
          ELSE IF(IRM1.GE.1 .AND. IRM1.LE.NRP  .AND.
     &       IWM2.GE.1 .AND. IWM2.LE.NWP        ) THEN
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
          ELSE
            STOP 'Cannot start in corner and go in'
          ENDIF
c
          if(wr.le.wrmax .and. ir.ge.nrp-2 .and. iw.ge.2) then
            ar0 = ar(ir-2,iw-1)
            ai0 = ai(ir-2,iw-1)
          endif
C
          AR(IR,IW) = AR0
          AI(IR,IW) = AI0
C
          IF(WR .GT. WRMAX) GO TO 810
C
          ITMAX = 12
          CALL ORRS(LST,LRE,N,ETA,U,S, RT(IR), ITMAX,
     &              AR0,AI0, WR0,WI0, UTR,UTI, VTR,VTI, DELMAX)
C
          IF(DELMAX.GT.RESMAX) GO TO 810
C
          AR(IR,IW) = AR0
          AI(IR,IW) = AI0
C
  810   CONTINUE
   80 CONTINUE
C
C
      DO 90 IW=1, NWP
        WRITE(19) (AR(IR,IW),IR=1, NRP)
        WRITE(19) (AI(IR,IW),IR=1, NRP)
   90 CONTINUE
C
      CLOSE(19)
C
      STOP
      END




      SUBROUTINE GETFR(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IW1,IW2)
      DIMENSION RTL(NRP), WSL(NWP)
      DIMENSION AR(NRX,NWX), AI(NRX,NWX)
C
    3 WRITE(*,1300) (IW,WSL(IW), IW=1, NWP)
 1300 FORMAT(/1X,'  j  log[W*sqrt(Rth)]'
     &  1000(/1X, I3, 6X, F7.4)  )
C
    4 CALL ASKI('Select j of freq. to examine (0=list,-1=end)^',IW)
      IF(IW.EQ.-1) GO TO 9
      IF(IW.LE.0 .OR. IW.GT.NWP) GO TO 3
C
      WRITE(*,1340) (IR,RTL(IR),AR(IR,IW),AI(IR,IW), IR=1, NRP)
C                 112    2.3452     0.12345   -.00123
 1340 FORMAT(/1X,'  i  log(Rtheta)     ar        ai'
     &    81(/1X, I3, 3X, F7.4, 2X, 2F10.5)  )
      GO TO 4
C
    9 CONTINUE
      CALL ASKI('Specify first frequency index^',IW1)
      CALL ASKI('Specify last  frequency index^',IW2)
      RETURN
C
      END


      SUBROUTINE GETRE(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IR1,IR2)
      DIMENSION RTL(NRP), WSL(NWP)
      DIMENSION AR(NRX,NWX), AI(NRX,NWX)
C
    3 WRITE(*,1300) (IR,RTL(IR), IR=1, NRP)
 1300 FORMAT(/1X,'  j    log[Rtheta]'
     &  1000(/1X, I3, 6X, F7.4)  )
C
    4 CALL ASKI('Select i of Rtheta to examine (0=list,-1=end)^',IR)
      IF(IR.EQ.-1) GO TO 9
      IF(IR.LE.0 .OR. IR.GT.NRP) GO TO 3
C
      WRITE(*,1340) (IW,WSL(IW),AR(IR,IW),AI(IR,IW), IW=1, NWP)
C                 112       2.3452       0.12345   -.00123
 1340 FORMAT(/1X,'  i  log[W*sqrt(Rth)]     ar        ai'
     &    81(/1X, I3, 6X, F7.4, 4X, 2F10.5)  )
      GO TO 4
C
    9 CONTINUE
      CALL ASKR('Specify first Rtheta index^',IR1)
      CALL ASKR('Specify last  Rtheta index^',IR2)
      RETURN
C
      END


