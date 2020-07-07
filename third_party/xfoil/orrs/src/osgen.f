
      PROGRAM OSGEN
C-----------------------------------------------------------------------
C     Reads OS amplification data alpha(R,w) stored in separate files, 
C     one file for each H value.
C
C     Distills this data into arrays which define a tri-cubic spline
C     which can be efficiently interrogated to return the alpha(R,W,H)
C     function and its derivatives.
C
C     The tri-cubic spline data is written out as a binary file,
C     to be read and used in SUBROUTINE OSMAP.
C
C     Usage:
C
C       % osgen os_list_file
C
C-----------------------------------------------------------------------
C
      PARAMETER (NMAX=257,NRX=111,NWX=91,NHX=21)
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
C
      REAL ATMP(NRX+NWX+NHX), ADTMP(NRX+NWX+NHX)
      REAL AC(NRX,NWX,NHX,2),
     &   AC_R(NRX,NWX,NHX,2), AC_W(NRX,NWX,NHX,2), AC_H(NRX,NWX,NHX,2),
     &  AC_RW(NRX,NWX,NHX,2),AC_RH(NRX,NWX,NHX,2),AC_WH(NRX,NWX,NHX,2),
     & AC_RWH(NRX,NWX,NHX,2)
      REAL RTL(NRX,NHX), WSL(NWX,NHX), HHL(NHX)
      INTEGER N(NHX), NRP(NHX), NWP(NHX)
      INTEGER IRP1(NHX),IRP2(NHX),IWP1(NHX),IWP2(NHX)
C
      PARAMETER (NRZ=31, NWZ=41, NHZ=21)
      INTEGER IW1(NHZ),  IW2(NHZ),  IR1(NHZ),  IR2(NHZ)
      REAL RL(NRZ), WL(NWZ), HL(NHZ),
     &      A(NRZ,NWZ,NHZ,2),
     &     AR(NRZ,NWZ,NHZ,2), AW(NRZ,NWZ,NHZ,2), AH(NRZ,NWZ,NHZ,2),
     &    ARW(NRZ,NWZ,NHZ,2),ARH(NRZ,NWZ,NHZ,2),AWH(NRZ,NWZ,NHZ,2),
     &   ARWH(NRZ,NWZ,NHZ,2)
C
      CHARACTER*80 ARGP1
      LOGICAL LSPLINE
C
C---- if T, use splines to compute derivatives, otherwise use finite-diff.
      LSPLINE = .TRUE.
C
C---- strides in R and W file values selected for storage in binary table
C-    (i.e. binary table can be less dense than the source storage files)
      IRINC = 4
      IWINC = 2
C
      CALL GETARG0(1,ARGP1)
C
      IF(ARGP1 .EQ. ' ') THEN
       WRITE(*,*) 'Enter file containing list of OS datafiles'
       READ(*,'(A)') ARGP1
      ENDIF
C
C---- set expeced format of source files
      IFORM = -1 !  unknown
ccc   IFORM = 0  !  binary
ccc   IFORM = 1  !  ascii
C
      CALL READOS(ARGP1,IFORM,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HHL, AC(1,1,1,1), AC(1,1,1,2),
     &            NRX,NWX,NHX)
C
C
      RTLMIN = RTL(1,1)
      WSLMIN = WSL(1,1)
      RTLMAX = RTL(1,1)
      WSLMAX = WSL(1,1)
      DO IHP=1, NHP
        RTLMIN = MIN( RTLMIN , RTL(1,IHP) )
        WSLMIN = MIN( WSLMIN , WSL(1,IHP) )
        RTLMAX = MAX( RTLMAX , RTL(NRP(IHP),IHP) )
        WSLMAX = MAX( WSLMAX , WSL(NWP(IHP),IHP) )
      ENDDO
C
      DRTL = RTL(2,1) - RTL(1,1)
      DWSL = WSL(2,1) - WSL(1,1)
C
      NRPTOT = INT( (RTLMAX - RTLMIN)/DRTL + 1.001 )
      NWPTOT = INT( (WSLMAX - WSLMIN)/DWSL + 1.001 )
C
      IF(NRPTOT .GT. NRX) STOP 'OSGEN: R index overflow'
      IF(NWPTOT .GT. NWX) STOP 'OSGEN: W index overflow'
C
C---- move ar,ai array for each H to a common origin for splining
      DO 20 IHP=1, NHP
        IROFF = INT( (RTL(1,IHP) - RTLMIN)/DRTL + 0.001 )
        IWOFF = INT( (WSL(1,IHP) - WSLMIN)/DWSL + 0.001 )
        IF(IROFF.EQ.0 .AND. IWOFF.EQ.0) GO TO 19
C
        DO IC = 1, 2
        DO IRP=NRP(IHP), 1, -1
          DO IWP=NWP(IHP), 1, -1
            AC(IRP+IROFF,IWP+IWOFF,IHP,IC) = AC(IRP,IWP,IHP,IC)
            AC(IRP,IWP,IHP,IC) = 0.0
          ENDDO
        ENDDO
        ENDDO
C
        IF(IROFF.GT.0) THEN
         DO IRP=NRP(IHP), 1, -1
           RTL(IRP+IROFF,IHP) = RTL(IRP,IHP)
           RTL(IRP,IHP) = 0.0
         ENDDO
        ENDIF
C
        IF(IWOFF.GT.0) THEN
         DO IWP=NWP(IHP), 1, -1
           WSL(IWP+IWOFF,IHP) = WSL(IWP,IHP)
           WSL(IWP,IHP) = 0.0
         ENDDO
        ENDIF
C
   19   IRP1(IHP) = IROFF + 1
        IWP1(IHP) = IWOFF + 1
        IRP2(IHP) = IROFF + NRP(IHP)
        IWP2(IHP) = IWOFF + NWP(IHP)
C
C------ set newly-defined R and W coordinate values
        DO IRP=1, IRP1(IHP)-1
          RTL(IRP,IHP) = RTL(IRP1(IHP),IHP) + DRTL*FLOAT(IRP-IRP1(IHP))
        ENDDO
        DO IRP=IRP2(IHP)+1, NRPTOT
          RTL(IRP,IHP) = RTL(IRP2(IHP),IHP) + DRTL*FLOAT(IRP-IRP2(IHP))
        ENDDO
C
        DO IWP=1, IWP1(IHP)-1
          WSL(IWP,IHP) = WSL(IWP1(IHP),IHP) + DWSL*FLOAT(IWP-IWP1(IHP))
        ENDDO
        DO IWP=IWP2(IHP)+1, NWPTOT
          WSL(IWP,IHP) = WSL(IWP2(IHP),IHP) + DWSL*FLOAT(IWP-IWP2(IHP))
        ENDDO
C
   20 CONTINUE
C
C---- differentiate in H with spline routine to get AC_H
      DO 40 IRP=1, NRPTOT
        DO 401 IWP=1, NWPTOT
C
C-------- find first H index at this R,w
          DO IHP=1, NHP
            IF(IRP.GE.IRP1(IHP) .AND. IRP.LE.IRP2(IHP) .AND.
     &         IWP.GE.IWP1(IHP) .AND. IWP.LE.IWP2(IHP) ) GO TO 4012
          ENDDO
          GO TO 401
 4012     IHP1 = IHP
C
C-------- find last H index at this R,w
          DO IHP=NHP, 1, -1
            IF(IRP.GE.IRP1(IHP) .AND. IRP.LE.IRP2(IHP) .AND.
     &         IWP.GE.IWP1(IHP) .AND. IWP.LE.IWP2(IHP) ) GO TO 4022
          ENDDO
          GO TO 401
 4022     IHP2 = IHP
C

          DO IC = 1, 2
            DO IHP=IHP1, IHP2
              ATMP(IHP) = AC(IRP,IWP,IHP,IC)
            ENDDO
C
            IHPNUM = IHP2 - IHP1 + 1

c           if(ihpnum.le.1) then
c             write(*,*) irp, iwp, ihp1, ihp2
c             write(*,*) rtl(irp,ihp), wsl(iwp,ihp), hhl(ihp)
c            stop
c           endif

            IF(IHPNUM .GT. 1) THEN
             CALL SPLINE(ATMP(IHP1),ADTMP(IHP1),HHL(IHP1),IHPNUM)
            ENDIF
C
            DO IHP=IHP1, IHP2
              AC_H(IRP,IWP,IHP,IC) = ADTMP(IHP)
            ENDDO
          ENDDO
C
  401   CONTINUE
   40 CONTINUE
C
C
      DO 50 IC = 1, 2
        IF(LSPLINE) THEN
C------- calculate AC_R and AC_W arrays from spline coefficients
         CALL RDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX, 
     &               AC(1,1,1,IC), AC_R(1,1,1,IC) )
         CALL WDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC(1,1,1,IC), AC_W(1,1,1,IC) )
C
         CALL RDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX,
     &               AC_H(1,1,1,IC), AC_RH(1,1,1,IC) )
         CALL WDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_H(1,1,1,IC), AC_WH(1,1,1,IC) )
         CALL WDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_R(1,1,1,IC), AC_RW(1,1,1,IC) )
C
         CALL WDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_RH(1,1,1,IC), AC_RWH(1,1,1,IC) )
C
        ELSE
C------- calculate AC_R and AC_W arrays by finite-differencing
         CALL RDIFF(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX, 
     &               AC(1,1,1,IC), AC_R(1,1,1,IC) )
         CALL WDIFF(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC(1,1,1,IC), AC_W(1,1,1,IC) )
C
         CALL RDIFF(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX,
     &               AC_H(1,1,1,IC), AC_RH(1,1,1,IC) )
         CALL WDIFF(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_H(1,1,1,IC), AC_WH(1,1,1,IC) )
         CALL WDIFF(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_R(1,1,1,IC), AC_RW(1,1,1,IC) )
C
         CALL WDIFF(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &               AC_RH(1,1,1,IC), AC_RWH(1,1,1,IC) )
        ENDIF
 50   CONTINUE
C
C
C---- set coarsened array limits
      NR = (NRPTOT-1)/IRINC + 1
      NW = (NWPTOT-1)/IWINC + 1
      NH =  NHP
C
      DO 60 IHP=1, NHP
        IH = IHP
        IR1(IH) = (IRP1(IHP)-1)/IRINC + 1
        IR2(IH) = (IRP2(IHP)-1)/IRINC + 1
        IW1(IH) = (IWP1(IHP)-1)/IWINC + 1
        IW2(IH) = (IWP2(IHP)-1)/IWINC + 1
C
        DO IR=1, NR
          IRP = IRINC*(IR-1) + 1
          DO IW=1, NW
            IWP = IWINC*(IW-1) + 1
            DO IC = 1, 2
              A   (IR,IW,IH,IC) = AC    (IRP,IWP,IHP,IC)
              AR  (IR,IW,IH,IC) = AC_R  (IRP,IWP,IHP,IC)
              AW  (IR,IW,IH,IC) = AC_W  (IRP,IWP,IHP,IC)
              AH  (IR,IW,IH,IC) = AC_H  (IRP,IWP,IHP,IC)
              ARW (IR,IW,IH,IC) = AC_RW (IRP,IWP,IHP,IC)
              ARH (IR,IW,IH,IC) = AC_RH (IRP,IWP,IHP,IC)
              AWH (IR,IW,IH,IC) = AC_WH (IRP,IWP,IHP,IC)
              ARWH(IR,IW,IH,IC) = AC_RWH(IRP,IWP,IHP,IC)
            ENDDO
          ENDDO
        ENDDO
   60 CONTINUE
C
C---- also set coarsened independent vaiable arrays
      IHP = 1
C
      DO IR=1, NR
        IRP = IRINC*(IR-1) + 1
        RL(IR) = RTL(IRP,IHP)
      ENDDO
C
      DO IW=1, NW
        IWP = IWINC*(IW-1) + 1
        WL(IW) = WSL(IWP,IHP)
      ENDDO
C
      DO IH=1, NH
        IHP = IH
        HL(IH) = HHL(IHP)
      ENDDO
C
C
C---- write coarsened arrays into binary data file
      LU = 30
      OPEN(LU,FILE='osmap.dat',STATUS='UNKNOWN',FORM='UNFORMATTED')
C
      WRITE(*,*) 'Writing  osmap.dat ...'
C
      WRITE(LU) NR, NW, NH
      WRITE(LU) (RL(IR), IR=1,NR)
      WRITE(LU) (WL(IW), IW=1,NW)
      WRITE(LU) (HL(IH), IH=1,NH)
      WRITE(LU) (IR1(IH),IR2(IH),IW1(IH),IW2(IH), IH=1,NH)
C
C---- write ai first, then ar
      DO 70 IC = 2, 1, -1
        DO IH=1, NH
          DO IW=IW1(IH), IW2(IH)
            WRITE(LU) (   A(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) (  AR(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) (  AW(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) (  AH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) ( ARW(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) ( ARH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) ( AWH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
            WRITE(LU) (ARWH(IR,IW,IH,IC), IR=IR1(IH),IR2(IH))
          ENDDO
        ENDDO
 70   CONTINUE
C
      CLOSE(LU)
C
      STOP
      END




      SUBROUTINE RDIFF(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX,
     &                  AC, AC_R )
      REAL AC(NRX,NWX,*),AC_R(NRX,NWX,*)
      REAL RTL(NRX,*)
      INTEGER IRP1(*),IRP2(*),IWP1(*),IWP2(*)
C
      DO 1 IHP=1, NHP
C
C------ differentiate in R with finite differences
        DO 10 IWP=IWP1(IHP), IWP2(IHP)
          IRP = IRP1(IHP)
          DELR = RTL(IRP+1,IHP) - RTL(IRP,IHP)
          AC_R(IRP,IWP,IHP) = (-3.0*AC(IRP  ,IWP,IHP)
     &                        + 4.0*AC(IRP+1,IWP,IHP)
     &                        -     AC(IRP+2,IWP,IHP) )/DELR
          IRP = IRP2(IHP)
          DELR = RTL(IRP,IHP) - RTL(IRP-1,IHP)
          AC_R(IRP,IWP,IHP) = ( 3.0*AC(IRP  ,IWP,IHP)
     &                        - 4.0*AC(IRP-1,IWP,IHP)
     &                        +     AC(IRP-2,IWP,IHP) )/DELR
          DO 101 IRP=IRP1(IHP)+1, IRP2(IHP)-1
            DELR = RTL(IRP+1,IHP) - RTL(IRP-1,IHP)
            AC_R(IRP,IWP,IHP) = ( AC(IRP+1,IWP,IHP)
     &                          - AC(IRP-1,IWP,IHP) )/DELR
  101     CONTINUE
   10   CONTINUE
C
    1 CONTINUE
C
      RETURN
      END



      SUBROUTINE WDIFF(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &                  AC, AC_W)
      REAL AC(NRX,NWX,*),AC_W(NRX,NWX,*)
      REAL WSL(NWX,*)
      INTEGER IRP1(*),IRP2(*),IWP1(*),IWP2(*)
C
      DO 1 IHP=1, NHP
C
C------ differentiate in F with finite differences
        DO 10 IRP=IRP1(IHP), IRP2(IHP)
          IWP = IWP1(IHP)
          DELF = WSL(IWP+1,IHP) - WSL(IWP,IHP)
          AC_W(IRP,IWP,IHP) = (-3.0*AC(IRP,IWP  ,IHP)
     &                        + 4.0*AC(IRP,IWP+1,IHP)
     &                        -     AC(IRP,IWP+2,IHP) )/DELF
          IWP = IWP2(IHP)
          DELF = WSL(IWP,IHP) - WSL(IWP-1,IHP)
          AC_W(IRP,IWP,IHP) = ( 3.0*AC(IRP,IWP  ,IHP)
     &                        - 4.0*AC(IRP,IWP-1,IHP)
     &                        +     AC(IRP,IWP-2,IHP) )/DELF
          DO 101 IWP=IWP1(IHP)+1, IWP2(IHP)-1
            DELF = WSL(IWP+1,IHP) - WSL(IWP-1,IHP)
            AC_W(IRP,IWP,IHP) = ( AC(IRP,IWP+1,IHP)
     &                          - AC(IRP,IWP-1,IHP) )/DELF
  101     CONTINUE
   10   CONTINUE
C
    1 CONTINUE
C
      RETURN
      END



      SUBROUTINE RDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,RTL,NRX,NWX,
     &                  AC, AC_R )
      REAL AC(NRX,NWX,*),AC_R(NRX,NWX,*)
      REAL RTL(NRX,*)
      INTEGER IRP1(*),IRP2(*),IWP1(*),IWP2(*)
C
      PARAMETER (NDIM=500)
      REAL ATMP(NDIM), ADTMP(NDIM)
C
      DO 1 IHP=1, NHP
        IF(IRP2(IHP).GT.NDIM) THEN
         WRITE(*,*) 'RDIFFS: Array overflow. Increase NDIM to',IRP2(IHP)
         STOP
        ENDIF
C
C------ differentiate in R with spline
        DO 10 IWP=IWP1(IHP), IWP2(IHP)
C
          DO 101 IRP=IRP1(IHP), IRP2(IHP)
            ATMP(IRP) = AC(IRP,IWP,IHP)
  101     CONTINUE
C
          IRP = IRP1(IHP)
          NUM = IRP2(IHP) - IRP1(IHP) + 1
          CALL SPLINE(ATMP(IRP),ADTMP(IRP),RTL(IRP,IHP),NUM)
C
          DO 102 IRP=IRP1(IHP), IRP2(IHP)
            AC_R(IRP,IWP,IHP) = ADTMP(IRP)
  102     CONTINUE
C
   10   CONTINUE
C
    1 CONTINUE
C
      RETURN
      END



      SUBROUTINE WDIFFS(IRP1,IRP2,IWP1,IWP2,NHP,WSL,NRX,NWX,
     &                  AC, AC_W)
      REAL AC(NRX,NWX,*),AC_W(NRX,NWX,*)
      REAL WSL(NWX,*)
      INTEGER IRP1(*),IRP2(*),IWP1(*),IWP2(*)

      PARAMETER (NDIM=500)
      REAL ATMP(NDIM), ADTMP(NDIM)
C
      DO 1 IHP=1, NHP
        IF(IWP2(IHP).GT.NDIM) THEN
         WRITE(*,*) 'WDIFFS: Array overflow. Increase NDIM to',IWP2(IHP)
         STOP
        ENDIF
C
C------ differentiate in F with spline
        DO 10 IRP=IRP1(IHP), IRP2(IHP)
C
          DO 101 IWP=IWP1(IHP), IWP2(IHP)
            ATMP(IWP) = AC(IRP,IWP,IHP)
  101     CONTINUE
C
          IWP = IWP1(IHP)
          NUM = IWP2(IHP) - IWP1(IHP) + 1
          CALL SPLINE(ATMP(IWP),ADTMP(IWP),WSL(IWP,IHP),NUM)
C
          DO 102 IWP=IWP1(IHP), IWP2(IHP)
            AC_W(IRP,IWP,IHP) = ADTMP(IWP)
  102     CONTINUE
C
   10   CONTINUE
C
    1 CONTINUE
C
      RETURN
      END
