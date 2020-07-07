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
      RESMAX = 0.01
C
      CALL READIT(N,H,ETA,U,S,NRP,NWP,RTL,WSL,AR,AI,NRX,NWX)
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
      WRITE(6,1200) RTL(1), RTL(NRP), NR, WSL(1), WSL(NWP), NW
 1200 FORMAT(/' log(Rth) :  low =', F7.4,'   high =', F7.4,'   NR =',I3
     &       /' log(W*sR):  low =', F7.4,'   high =', F7.4,'   NW =',I3)
C
      WRITE(6,*) ' '
      WRITE(6,*) '1  Add/replace scaled frequencies'
      WRITE(6,*) '2  Add/replace Reynolds numbers'
      WRITE(6,*) ' '
      CALL ASK('Select option^',2,IOPT)
      WRITE(6,*) ' '
C
      IF(IOPT.EQ.1) THEN
C
C----- get starting and final frequency indices
       CALL GETFR(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IW1,IW2)
       IWINCR = ISIGN( 1 , (IW2-IW1) )
C
       CALL ASK('Enter Re index (+/- dir) to start at^',2,IR1S)
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
       CALL ASK('Enter W index (+/- dir) to start at^',2,IW1S)
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
c
      iw0 = iw1 - 3*iwincr
      ar_iw  = (ar(ir2,iw0+2)-ar(ir2,iw0-2))/4.0
      ai_iw  = (ai(ir2,iw0+2)-ai(ir2,iw0-2))/4.0
      ar2_iw = (ar(ir2,iw0+2)+ar(ir2,iw0-2)-2.0*ar(ir2,iw0))/4.0
      ai2_iw = (ai(ir2,iw0+2)+ai(ir2,iw0-2)-2.0*ai(ir2,iw0))/4.0
c
      ir0 = ir1 - 3*irincr
      ar_ir  = (ar(ir0+2,iw2)-ar(ir0-2,iw2))/4.0
      ai_ir  = (ai(ir0+2,iw2)-ai(ir0-2,iw2))/4.0
      ar2_ir = (ar(ir0+2,iw2)+ar(ir0-2,iw2)-2.0*ar(ir0,iw2))/4.0
      ai2_ir = (ai(ir0+2,iw2)+ai(ir0-2,iw2)-2.0*ai(ir0,iw2))/4.0
c
      iw = iw2
      arneww = ar(ir2,iw0) + ar_iw*(iw-iw0) + ar2_iw*0.5*(iw-iw0)**2
      aineww = ai(ir2,iw0) + ai_iw*(iw-iw0) + ai2_iw*0.5*(iw-iw0)**2
c
      ir = ir2
      arnewr = ar(ir0,iw2) + ar_ir*(ir-ir0) + ar2_ir*0.5*(ir-ir0)**2
      ainewr = ai(ir0,iw2) + ai_ir*(ir-ir0) + ai2_ir*0.5*(ir-ir0)**2
c
      ardif = (arneww - arnewr) * 0.5
      aidif = (aineww - ainewr) * 0.5
c
      ar2_iw = ar2_iw - ardif*2.0/(iw-iw0)**2
      ai2_iw = ai2_iw - aidif*2.0/(iw-iw0)**2
c
      ar2_ir = ar2_ir + ardif*2.0/(ir-ir0)**2
      ai2_ir = ai2_ir + aidif*2.0/(ir-ir0)**2
c
      do iw=iw1, iw2, iwincr
        arnew = ar(ir2,iw0) + ar_iw*(iw-iw0) + ar2_iw*0.5*(iw-iw0)**2
        ainew = ai(ir2,iw0) + ai_iw*(iw-iw0) + ai2_iw*0.5*(iw-iw0)**2
        ar(ir2,iw) = arnew
        ai(ir2,iw) = ainew
      enddo
c
      do ir=ir1, ir2, irincr
        arnew = ar(ir0,iw2) + ar_ir*(ir-ir0) + ar2_ir*0.5*(ir-ir0)**2
        ainew = ai(ir0,iw2) + ai_ir*(ir-ir0) + ai2_ir*0.5*(ir-ir0)**2
        ar(ir,iw2) = arnew
        ai(ir,iw2) = ainew
      enddo
c
C---------------------
C
      CALL ASK('Enter map output filename^',4,FNAME)
      OPEN(19,FILE=FNAME,STATUS='NEW',FORM='UNFORMATTED')
      WRITE(19) N, H
      WRITE(19) (ETA(I),I=1, N)
      WRITE(19) (U(I)  ,I=1, N)
      WRITE(19) (S(I)  ,I=1, N)
      WRITE(19)  NRP, NWP
      WRITE(19) (RTL(IR),IR=1,NRP)
      WRITE(19) (WSL(IW),IW=1,NWP)
C
      do ipass=1, 300
c
      DO 80 IW=IW1, IW2, IWINCR
C
ccc        WRITE(6,2010)
 2010   FORMAT(/1X,'--------------------')
        DO 810 IR=IR1, IR2, IRINCR
C
          WR = WS(IW)/SQRT(RT(IR))
C
ccc       WRITE(6,2020) IW,IR, RT(IR), WR
 2020     FORMAT(/1X,2I4,'    Rth =', E12.4, '    Wr =', E12.4)
C
          WR0 = WR
          WI0 = 0.0
C
CCC          IF(IOPT.EQ.1) THEN
ccc           AR0 = 2.0*AR(IR,IW-IWINCR) - AR(IR,IW-2*IWINCR)
ccc           AI0 = 2.0*AI(IR,IW-IWINCR) - AI(IR,IW-2*IWINCR)
CCC          ELSE IF(IOPT.EQ.2) THEN
ccc           AR0 = 2.0*AR(IR-IRINCR,IW) - AR(IR-2*IRINCR,IW) + AR0
ccc           AI0 = 2.0*AI(IR-IRINCR,IW) - AI(IR-2*IRINCR,IW) + AI0
CCC          ENDIF
          if(ir.eq.ir2 .or. iw.eq.iw2) go to 810
c
           AR(IR,IW) = ( ar(ir,iw-1) + ar(ir,iw+1)
     &                 + ar(ir-1,iw) + ar(ir+1,iw) ) * 0.25
           AI(IR,IW) = ( ai(ir,iw-1) + ai(ir,iw+1)
     &                 + ai(ir-1,iw) + ai(ir+1,iw) ) * 0.25
          if(.true.) go to 810
C
C
          ITMAX = 12
          CALL ORRS(LST,LRE,N,ETA,U,S, RT(IR), ITMAX,
     &              AR0,AI0, WR0,WI0, UTR,UTI, VTR,VTI, DELMAX)
C
          IF(DELMAX.GT.RESMAX) THEN
           IF(IOPT.EQ.1) THEN
            AR0 = 2.0*AR(IR,IW-IWINCR) - AR(IR,IW-2*IWINCR)
            AI0 = 2.0*AI(IR,IW-IWINCR) - AI(IR,IW-2*IWINCR)
           ELSE IF(IOPT.EQ.2) THEN
            AR0 = 2.0*AR(IR-IRINCR,IW) - AR(IR-2*IRINCR,IW)
            AI0 = 2.0*AI(IR-IRINCR,IW) - AI(IR-2*IRINCR,IW)
           ENDIF
          ENDIF
C
          AR(IR,IW) = AR0
          AI(IR,IW) = AI0
C
  810   CONTINUE
   80 CONTINUE
c
      enddo
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



      SUBROUTINE READIT(N,H,ETA,U,S,NRP,NWP,RTL,WSL,AR,AI,NRX,NWX)
      DIMENSION ETA(1), U(1), S(1)
      DIMENSION AR(NRX,NWX), AI(NRX,NWX)
      DIMENSION RTL(NRX), WSL(NWX)
      LOGICAL*1 FNAME(32)
C
      CALL ASK('Enter map filename^',4,FNAME)
      OPEN(9,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED')
C
      READ(9) N, H
      READ(9) (ETA(I),I=1, N)
      READ(9) (U(I)  ,I=1, N)
      READ(9) (S(I)  ,I=1, N)
      READ(9) NRP, NWP
      READ(9) (RTL(IR),IR=1,NRP)
      READ(9) (WSL(IW),IW=1,NWP)
C
      DO 10 IW=1, NWP
        READ(9,END=11) (AR(IR,IW),IR=1,NRP)
        READ(9,END=11) (AI(IR,IW),IR=1,NRP)
   10 CONTINUE
      CLOSE(9)
      GO TO 90
C
   11 CONTINUE
      CLOSE(9)
      NWP = IW-1
      WRITE(6,*) 'Map incomplete.'
      WRITE(6,*) 'Last complete frequency index set:',NWP
C
   90 CONTINUE
      GEO = (ETA(3)-ETA(2)) / (ETA(2)-ETA(1))
C
      WRITE(6,1050) N, H, ETA(N), GEO
 1050 FORMAT(/' n =', I4,'   H =', F7.3,
     &                   '   Ye =', F7.3,
     &                   '   dYi+1/dYi =',F6.3 /)
C
      RETURN
      END



      SUBROUTINE GETFR(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IW1,IW2)
      DIMENSION RTL(NRP), WSL(NWP)
      DIMENSION AR(NRX,NWX), AI(NRX,NWX)
C
    3 WRITE(6,1300) (IW,WSL(IW), IW=1, NWP)
 1300 FORMAT(/1X,'  j  log[W*sqrt(Rth)]'
     &  1000(/1X, I3, 6X, F7.4)  )
C
    4 CALL ASK('Select j of freq. to examine (0=list,-1=end)^',2,IW)
      IF(IW.EQ.-1) GO TO 9
      IF(IW.LE.0 .OR. IW.GT.NWP) GO TO 3
C
      WRITE(6,1340) (IR,RTL(IR),AR(IR,IW),AI(IR,IW), IR=1, NRP)
C                 112    2.3452     0.12345   -.00123
 1340 FORMAT(/1X,'  i  log(Rtheta)     ar        ai'
     &    81(/1X, I3, 3X, F7.4, 2X, 2F10.5)  )
      GO TO 4
C
    9 CONTINUE
      CALL ASK('Specify first frequency index^',2,IW1)
      CALL ASK('Specify last  frequency index^',2,IW2)
      RETURN
C
      END


      SUBROUTINE GETRE(NRP,NWP,RTL,WSL,AR,AI,NRX,NWX, IR1,IR2)
      DIMENSION RTL(NRP), WSL(NWP)
      DIMENSION AR(NRX,NWX), AI(NRX,NWX)
C
    3 WRITE(6,1300) (IR,RTL(IR), IR=1, NRP)
 1300 FORMAT(/1X,'  j    log[Rtheta]'
     &  1000(/1X, I3, 6X, F7.4)  )
C
    4 CALL ASK('Select i of Rtheta to examine (0=list,-1=end)^',2,IR)
      IF(IR.EQ.-1) GO TO 9
      IF(IR.LE.0 .OR. IR.GT.NRP) GO TO 3
C
      WRITE(6,1340) (IW,WSL(IW),AR(IR,IW),AI(IR,IW), IW=1, NWP)
C                 112       2.3452       0.12345   -.00123
 1340 FORMAT(/1X,'  i  log[W*sqrt(Rth)]     ar        ai'
     &    81(/1X, I3, 6X, F7.4, 4X, 2F10.5)  )
      GO TO 4
C
    9 CONTINUE
      CALL ASK('Specify first Rtheta index^',2,IR1)
      CALL ASK('Specify last  Rtheta index^',2,IR2)
      RETURN
C
      END


