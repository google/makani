
      SUBROUTINE READOS(FLIST,IFORM,
     &                  N,NMAX,ETA,U,S,
     &                  NRP,NWP,NHP,
     &                  RTL,WSL,HH , AR,AI,
     &                  NRX,NWX,NHX)
C----------------------------------------------------------------
C     Reads Orr-Sommerfeld data files in binary or ascii format.
C     Data is spatial amplification complex wavenumber
C           ar(Re,w,H)  ai(Re,w,H)
C     stored on a R,W,H grid
C           R = ln(Re)
C           W = ln(w) - 0.5 ln(Re)
C           H = H
C
C     Input
C       FLIST   name of text file containing file prefixes to be read
C       IFORM   -1=unknown
C                0=binary
C                1=ascii
C
C     Output
C       N(h)      number of points across BL,  i=1..N
C       NMAX      max dimension of N
C       ETA(i,h)  BL y coordinate
C       U(i,h)    velocity profile
C       S(i,h)    shear profile dU/deta
C       NRP(h)    number of RTL values, r=1..NRP
C       NWP(h)    number of WSL values, w=1..NWP
C       NHP       number of H   values, h=1..NHP
C       RTL(r,h)  R  values
C       WSL(w,h)  W  values
C       HH(h)     H  values
C       AR(r,w,h) real wavenumber
C       AI(r,w,h) imaginary wavenumber
C
C----------------------------------------------------------------
      CHARACTER*(*) FLIST
      DIMENSION N(NHX), NRP(NHX),NWP(NHX)
      DIMENSION ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      DIMENSION AR(NRX,NWX,NHX),AI(NRX,NWX,NHX)
      DIMENSION RTL(NRX,NHX), WSL(NWX,NHX), HH(NHX)
      CHARACTER*80 FNAME
C
      OPEN(10,FILE=FLIST,STATUS='OLD')
C
      WRITE(*,*) 'Reading...'
      DO 1000 IH=1, NHX
 5      READ(10,5000,END=1001) FNAME
 5000   FORMAT(A)
C
C------ skip comment line
        IF(INDEX('#!',FNAME(1:1)) .NE. 0) GO TO 5
C
C------ strip off leading blanks
 10     CONTINUE
        IF(FNAME(1:1).EQ.' ') THEN
         FNAME = FNAME(2:80)
         GO TO 10
        ENDIF
C
        CALL READOS1(FNAME,IFORM,
     &               N(IH),NMAX,ETA(1,IH),U(1,IH),S(1,IH),
     &               NRP(IH),NWP(IH),
     &               RTL(1,IH),WSL(1,IH),HH(IH), AR(1,1,IH),AI(1,1,IH),
     &               NRX,NWX)
C
 1000 CONTINUE
      IH = NHX + 1
C
 1001 NHP = IH-1
      CLOSE(10)
C
      RETURN
      END ! READOS



      SUBROUTINE WRITOS(FLIST,IFORM,
     &                  N,NMAX,ETA,U,S,
     &                  NRP,NWP,NHP,
     &                  RTL,WSL,HH , AR,AI,
     &                  NRX,NWX,NHX)
C----------------------------------------------------------------
C     Writes Orr-Sommerfeld data files in binary or ascii format.
C     Data is spatial amplification complex wavenumber
C           ar(Re,w,H)  ai(Re,w,H)
C     stored on a R,W,H grid
C           R = ln(Re)
C           W = ln(w) - 0.5 ln(Re)
C           H = H
C
C     Input
C       FLIST   name of text file containing file prefixes to be read
C       IFORM   0=binary, ascii otherwise
C       N(h)      number of points across BL,  i=1..N
C       NMAX      max dimension of N
C       ETA(i,h)  BL y coordinate
C       U(i,h)    velocity profile
C       S(i,h)    shear profile dU/deta
C       NRP(h)    number of RTL values, r=1..NRP
C       NWP(h)    number of WSL values, w=1..NWP
C       NHP       number of H   values, h=1..NHP
C       RTL(r,h)  R  values
C       WSL(w,h)  W  values
C       HH(h)     H  values
C       AR(r,w,h) real wavenumber
C       AI(r,w,h) imaginary wavenumber
C
C     Output
C       written files
C
C----------------------------------------------------------------
C
      CHARACTER*(*) FLIST
      DIMENSION N(NHX), NRP(NHX),NWP(NHX)
      DIMENSION ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      DIMENSION AR(NRX,NWX,NHX),AI(NRX,NWX,NHX)
      DIMENSION RTL(NRX,NHX), WSL(NWX,NHX), HH(NHX)
      CHARACTER*80 FNAME
C
      OPEN(10,FILE=FLIST,STATUS='OLD')
C
      WRITE(*,*) 'Writing...'
      DO 1000 IH=1, NHX
 5      READ(10,5000,END=1001) FNAME
 5000   FORMAT(A)
C
C------ skip comment line
        IF(INDEX('#!',FNAME(1:1)) .NE. 0) GO TO 5
C
C------ strip off leading blanks
 10     CONTINUE
        IF(FNAME(1:1).EQ.' ') THEN
         FNAME = FNAME(2:80)
         GO TO 10
        ENDIF
C
        CALL WRITOS1(FNAME,IFORM,
     &               N(IH),NMAX,ETA(1,IH),U(1,IH),S(1,IH),
     &               NRP(IH),NWP(IH),
     &               RTL(1,IH),WSL(1,IH),HH(IH), AR(1,1,IH),AI(1,1,IH),
     &               NRX,NWX)
C
 1000 CONTINUE
      IH = NHX + 1
C
 1001 NHP = IH-1
      CLOSE(10)
      CLOSE(9)
C
      RETURN
      END ! WRITOS



      SUBROUTINE READOS1(FNAME,IFORM,
     &                   N,NMAX,ETA,U,S,
     &                   NRP,NWP,
     &                   RTL,WSL,HH , AR, AI,
     &                   NRX,NWX)
C----------------------------------------------------------------
C     Reads Orr-Sommerfeld data file in binary or ascii format.
C     Data is spatial amplification complex wavenumber
C           ar(Re,w,H)  ai(Re,w,H)
C     stored on a R,W,H grid
C           R = ln(Re)
C           W = ln(w) - 0.5 ln(Re)
C           H = H
C
C     Input
C       FNAME   name of data file to be read
C       IFORM   -1=unknown
C                0=binary
C                1=ascii
C
C     Output
C       N       number of points across BL,  i=1..N
C       NMAX      max dimension of N
C       ETA(i)  BL y coordinate
C       U(i)    velocity profile
C       S(i)    shear profile dU/deta
C       NRP    number of RTL values, r=1..NRP
C       NWP    number of WSL values, w=1..NWP
C       RTL(r)  R  values
C       WSL(w)  W  values
C       HH      H  value
C       AR(r,w) real wavenumber
C       AI(r,w) imaginary wavenumber
C
C----------------------------------------------------------------
      CHARACTER*(*) FNAME
      REAL ETA(NMAX), U(NMAX), S(NMAX)
      REAL AR(NRX,NWX),AI(NRX,NWX)
      REAL RTL(NRX), WSL(NWX)
C
      IF(IFORM.LE.-1) THEN
C----- first assume it's an ascii file
       KFORM = 1
C
C----- try reading it as a binary
       OPEN(9,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=1001)
       READ(9,ERR=11) NTEST, HTEST
C
       IF(NTEST.GE.1 .AND. NTEST.LE.NMAX) THEN
C------ point index within bounds... looks like it's binary
        KFORM = 0
       ENDIF
C
 11    CLOSE(9)
C
      ELSE
       KFORM = IFORM
C
      ENDIF
C
C
      K = INDEX(FNAME,' ') - 1
C
      IF(KFORM.EQ.0) THEN
C----- binary format
       FNAME = FNAME(1:K)
       WRITE(*,*) FNAME, '   binary'
C
       OPEN(9,FILE=FNAME,STATUS='OLD',FORM='UNFORMATTED',ERR=1001)
       READ(9,ERR=1001) N, HH
       READ(9) (ETA(I),I=1, N)
       READ(9) (U(I)  ,I=1, N)
       READ(9) (S(I)  ,I=1, N)
       READ(9) NRP, NWP
       READ(9) (RTL(IR),IR=1,NRP)
       READ(9) (WSL(IW),IW=1,NWP)
C
       DO IW=1, NWP
         READ(9,END=15) (AR(IR,IW),IR=1,NRP)
         READ(9,END=15) (AI(IR,IW),IR=1,NRP)
       ENDDO
       GO TO 30
C
 15    CONTINUE
       IWLAST = IW-1
       WRITE(*,*) 
     &     'Map incomplete.  Last complete frequency index:',IWLAST
C
      ELSE
C----- ascii format
       FNAME = FNAME(1:K)
       WRITE(*,*) FNAME, '   ascii'
C
       OPEN(9,FILE=FNAME,STATUS='OLD')
       READ(9,*) N, HH
       READ(9,*) (ETA(I),I=1, N)
       READ(9,*) (U(I)  ,I=1, N)
       READ(9,*) (S(I)  ,I=1, N)
       READ(9,*) NRP, NWP
       READ(9,*) (RTL(IR),IR=1,NRP)
       READ(9,*) (WSL(IW),IW=1,NWP)
C
       DO IW=1, NWP
         READ(9,*) (AR(IR,IW),IR=1,NRP)
         READ(9,*) (AI(IR,IW),IR=1,NRP)
       ENDDO
      ENDIF
C
 30   CONTINUE
      CLOSE(9)
      GEO = (ETA(3)-ETA(2)) / (ETA(2)-ETA(1))
      WRITE(*,2050) N, HH, ETA(N), GEO, 
     &              RTL(1), RTL(NRP), WSL(1), WSL(NWP)
 2050 FORMAT(' n =', I4,'   H =', F7.3,
     &                  '   Ye =', F7.3,
     &                  '   dYi+1/dYi =',F6.3,
     &                  '   Rt1 Rtn =', 2g12.4,
     &                  '   Wl1 Wln =', 2g12.4 /)
      IFORM = 1
C
C
C---- re-order if needed to make RTL and WSL monotonically increasing
      IF(RTL(1) .GT. RTL(NRP)) THEN
       DO IR=1, NRP/2
         IRBACK = NRP-IR+1
C
         RTEMP = RTL(IR)
         RTL(IR) = RTL(IRBACK)
         RTL(IRBACK) = RTEMP
C
         DO IW=1, NWP
           ARTEMP = AR(IR,IW)
           AITEMP = AI(IR,IW)
           AR(IR,IW) = AR(IRBACK,IW)
           AI(IR,IW) = AI(IRBACK,IW)
           AR(IRBACK,IW) = ARTEMP
           AI(IRBACK,IW) = AITEMP
         ENDDO
       ENDDO
      ENDIF
C
      IF(WSL(1) .GT. WSL(NWP)) THEN
       DO IW=1, NWP/2
         IWBACK = NWP-IW+1
C
         WTEMP = WSL(IW)
         WSL(IW) = WSL(IWBACK)
         WSL(IWBACK) = WTEMP
C
         DO IR=1, NRP
           ARTEMP = AR(IR,IW)
           AITEMP = AI(IR,IW)
           AR(IR,IW) = AR(IR,IWBACK)
           AI(IR,IW) = AI(IR,IWBACK)
           AR(IR,IWBACK) = ARTEMP
           AI(IR,IWBACK) = AITEMP
         ENDDO
       ENDDO
C
      ENDIF
      RETURN
C
 1001 CONTINUE
      WRITE(*,*) 'File open error.'
      RETURN
      END ! READOS1


      SUBROUTINE WRITOS1(FNAME,IFORM,
     &                   N,NMAX,ETA,U,S,
     &                   NRP,NWP,
     &                   RTL,WSL,HH , AR, AI,
     &                   NRX,NWX)
C----------------------------------------------------------------
C     Writes Orr-Sommerfeld data file in binary or ascii format.
C     Data is spatial amplification complex wavenumber
C           ar(Re,w,H)  ai(Re,w,H)
C     stored on a R,W,H grid
C           R = ln(Re)
C           W = ln(w) - 0.5 ln(Re)
C           H = H
C
C     Input
C       FNAME    name of data file to be written
C       IFORM    0=binary, ascii otherwise
C       N        number of points across BL,  i=1..N
C       NMAX     max dimension of N
C       ETA(i)   BL y coordinate
C       U(i)     velocity profile
C       S(i)     shear profile dU/deta
C       NRP      number of RTL values, r=1..NRP
C       NWP      number of WSL values, w=1..NWP
C       RTL(r)   R  values
C       WSL(w)   W  values
C       HH       H  value
C       AR(r,w)  real wavenumber
C       AI(r,w)  imaginary wavenumber
C
C     Output
C       written file
C
C----------------------------------------------------------------
      CHARACTER*(*) FNAME
      REAL ETA(NMAX), U(NMAX), S(NMAX)
      REAL AR(NRX,NWX),AI(NRX,NWX)
      REAL RTL(NRX), WSL(NWX)
C
      K = INDEX(FNAME,' ') - 1
C
      IF(IFORM.EQ.0) THEN
       FNAME = FNAME(1:K) // '.bin'
       WRITE(*,*) FNAME
C
       OPEN(9,FILE=FNAME,STATUS='UNKNOWN',FORM='UNFORMATTED',ERR=1001)
       REWIND(9)
       WRITE(9,ERR=1001) N, HH
       WRITE(9) (ETA(I),I=1, N)
       WRITE(9) (U(I)  ,I=1, N)
       WRITE(9) (S(I)  ,I=1, N)
       WRITE(9) NRP, NWP
       WRITE(9) (RTL(IR),IR=1,NRP)
       WRITE(9) (WSL(IW),IW=1,NWP)
C
       DO IW=1, NWP
         WRITE(9) (AR(IR,IW),IR=1,NRP)
         WRITE(9) (AI(IR,IW),IR=1,NRP)
       ENDDO
C
      ELSE
       FNAME = FNAME(1:K)
       WRITE(*,*) FNAME
C
       OPEN(9,FILE=FNAME,STATUS='UNKNOWN')
       REWIND(9)
       WRITE(9,*) N, HH
       WRITE(9,*) (ETA(I),I=1, N)
       WRITE(9,*) (U(I)  ,I=1, N)
       WRITE(9,*) (S(I)  ,I=1, N)
       WRITE(9,*) NRP, NWP
       WRITE(9,*) (RTL(IR),IR=1,NRP)
       WRITE(9,*) (WSL(IW),IW=1,NWP)
C
       DO IW=1, NWP
         WRITE(9,*) (AR(IR,IW),IR=1,NRP)
         WRITE(9,*) (AI(IR,IW),IR=1,NRP)
       ENDDO
      ENDIF
C
      CLOSE(9)
      RETURN
C
 1001 CONTINUE
      WRITE(*,*) 'File open error.'
      RETURN
      END ! WRITOS1
