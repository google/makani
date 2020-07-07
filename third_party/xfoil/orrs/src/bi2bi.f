      PROGRAM BI2BI
      PARAMETER (NMAX=257,NRX=101,NWX=91,NHX=21)
C
      REAL ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      REAL AR(NRX,NWX,NHX), AI(NRX,NWX,NHX)
      REAL RT(NRX,NHX),RTL(NRX,NHX)
      REAL WS(NWX,NHX),WSL(NWX,NHX)
      REAL HH(NHX),HHL(NHX)
      INTEGER N(NHX), NRP(NHX), NWP(NHX)
      CHARACTER*80 ARGP
C
      CALL GETARG(1,ARGP)
C
      CALL READIT(ARGP,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
C
      DO 1000 IH=1, NHX
        IF(N(IH) .EQ. 0) GO TO 1001
C
         CALL BREV(HH(IH))
C
         write(*,*) ih, n(ih), hh(ih)

         DO 10 I=1, N(IH)
           CALL BREV(ETA(I,IH))
           CALL BREV(U(I,IH))
           CALL BREV(S(I,IH))
 10      CONTINUE
C
         DO 20 IR=1, NRP(IH)
           CALL BREV(RTL(IR,IH))
 20      CONTINUE
C
         DO 30 IW=1, NWP(IH)
           CALL BREV(WSL(IW,IH))
 30      CONTINUE
C
         DO 40 IW=1, NWP(IH)
           DO 405 IR=1, NRP(IH)
             CALL BREV(AR(IR,IW,IH))
             CALL BREV(AI(IR,IW,IH))
 405       CONTINUE
 40      CONTINUE
C
         CALL BREV(N(IH))
         CALL BREV(NRP(IH))
         CALL BREV(NWP(IH))
C
 1000 CONTINUE
 1001 CONTINUE
C
C
      CALL DUMPIT(ARGP,
     &            N,NMAX,ETA,U,S,
     &            NRP,NWP,NHP,
     &            RTL,WSL,HH , AR,AI,
     &            NRX,NWX,NHX)
C
      STOP
      END


      SUBROUTINE DUMPIT(ARGP,
     &                  N,NMAX,ETA,U,S,
     &                  NRP,NWP,NHP,
     &                  RTL,WSL,HH , AR,AI,
     &                  NRX,NWX,NHX)
      CHARACTER*(*) ARGP
      DIMENSION N(NHX), NRP(NHX),NWP(NHX)
      DIMENSION ETA(NMAX,NHX), U(NMAX,NHX), S(NMAX,NHX)
      DIMENSION AR(NRX,NWX,NHX),AI(NRX,NWX,NHX)
      DIMENSION RTL(NRX,NHX), WSL(NWX,NHX), HH(NHX)
      CHARACTER*80 FNAME
C
      OPEN(10,FILE=ARGP,STATUS='OLD')
C
      DO 1000 IH=1, NHX
 5      READ(10,5000,END=1001) FNAME(2:80)
 5000   FORMAT(A)
C
C------ skip comment line
        IF(INDEX('#!',FNAME(1:1)) .NE. 0) GO TO 5
C
C------ strip off leading blanks
 10     CONTINUE
        IF(INDEX(FNAME(1:1).EQ.' ') THEN
         FNAME = FNAME(2:80)
         GO TO 10
        ENDIF
C
        K = INDEX(FNAME,' ') - 1
C
        FNAME = ARGP(1:K) // '.rbin'
        WRITE(*,*) FNAME
C
        OPEN(9,FILE=FNAME,STATUS='UNKNOWN',FORM='UNFORMATTED')
        WRITE(9) N(IH), HH(IH)
        CALL BREV(N(IH))

        HHREV = HH(IH)
        CALL BREV(HHREV)
C
        WRITE(9) (ETA(I,IH),I=1, N(IH))
        WRITE(9) (U(I,IH)  ,I=1, N(IH))
        WRITE(9) (S(I,IH)  ,I=1, N(IH))
        WRITE(9) NRP(IH), NWP(IH)
        CALL BREV(NRP(IH))
        CALL BREV(NWP(IH))
C
        WRITE(9) (RTL(IR,IH),IR=1,NRP(IH))
        WRITE(9) (WSL(IW,IH),IW=1,NWP(IH))
C
        DO IW=1, NWP(IH)
          WRITE(9) (AR(IR,IW,IH),IR=1,NRP(IH))
          WRITE(9) (AI(IR,IW,IH),IR=1,NRP(IH))
        ENDDO
        CLOSE(9)

        WRITE(*,*) N(IH), NRP(IH), NWP(IH), HHREV
C
 1000 CONTINUE
 1001 NHP = IH-1
      CLOSE(10)
      CLOSE(9)
C
      RETURN
      END


      SUBROUTINE BREV(AINP)

C---- byte-reverse between DEC and standard format
C
      LOGICAL*1 AB(4), TEMP
      EQUIVALENCE (A,AB)
C
ccc      return

      A = AINP
C
      TEMP = AB(1)
      AB(1) = AB(4)
      AB(4) = TEMP
C
      TEMP = AB(2)
      AB(2) = AB(3)
      AB(3) = TEMP
C
      AINP = A
C
      RETURN
      END
