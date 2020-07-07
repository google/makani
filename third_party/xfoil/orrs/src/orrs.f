
      SUBROUTINE ORRS(LSTI,LREI,NI,YI,UI,UDI, REI, ITMAXI,
     &                ALPHAR,ALPHAI , OMEGAR,OMEGAI, 
     &                UTR,UTI, VTR,VTI, WTR, WTI, CTR, CTI, DELMAX)
      DIMENSION YI(NI), UI(NI), UDI(NI)
      DIMENSION UTR(NI), UTI(NI), VTR(NI), VTI(NI),
     &          WTR(NI), WTI(NI), CTR(NI), CTI(NI)
C---------------------------------------------------------------------
C     Routine for solving the Orr-Sommerfeld equation
C     in the spatial or temporal stability problems.
C
C     Input:
C     ------
C      LSTI    1: spatial  amplification problem
C              2: temporal amplification problem
C      LREI    1: Reynolds number fixed
C              2: Reynolds number variable 
C                 [ to obtain specified ai (LSTI=1), or wi (LSTI=2) ]
C      NI      total number of points in profiles
C      YI      normal BL coordinate array
C      UI      mean flow u(y) profile
C      UDI     mean flow du/dy profile
C      REI     Reynolds number
C      ITMAXI  max number of Newton iterations to seek eigenvalue
C      OMEGAR  real  part of temporal frequency (for initial guess)
C      OMEGAI  imag. part of temporal frequency (for initial guess)
C      ALPHAR  real  part of complex wavenumber (for initial guess)
C      ALPHAI  imag. part of complex wavenumber (for initial guess)
C
C     Output:
C     -------
C      OMEGAR  real  part of temporal frequency (if LSTI = 2)
C      OMEGAI  imag. part of temporal frequency (if LSTI = 2)
C      ALPHAR  real  part of complex wavenumber (if LSTI = 1)
C      ALPHAI  imag. part of complex wavenumber (if LSTI = 1)
C      UTR     real  part of perturbation x-velocity profile
C      UTI     imag. part of perturbation x-velocity profile
C      VTR     real  part of perturbation y-velocity profile
C      VTI     imag. part of perturbation y-velocity profile
C      WTR     real  part of perturbation vorticity profile
C      WTI     imag. part of perturbation vorticity profile
C      CTR     real  part of perturbation d(vorticity)/dy profile
C      CTI     imag. part of perturbation d(vorticity)/dy profile
C      DELMAX  max change in (UTR,UTI) in last iteration (~= 0 if converged)
C---------------------------------------------------------------------
C
      INCLUDE 'ORRS.INC'
C
C---- convergence tolerance
      DATA EPS / 1.0E-4 /
C
      IF(NI.GT.NMAX) STOP 'ORRS: Array overflow.'
C
C---- set initial BC flag
      IBC  = 1
C
C---- set du'/dy normalization constant (normally imposed at wall)
      FNORM = (1.0,-1.0)
ccc      FNORM = (1.0, 0.0)
C
C---- set input variables from parameter list
      LST = LSTI
      LRE = LREI
C
      ITMAX = ITMAXI
C
      N = NI
      DO I=1, N
        Y(I) = YI(I)
        U(I) = UI(I)
        UD(I) = UDI(I)
      ENDDO
C
      IRE   = CMPLX( 0.0  , REI  )
      ALPHA = CMPLX(ALPHAR,ALPHAI)
      OMEGA = CMPLX(OMEGAR,OMEGAI)
C
C---- save initial guess for restoration if normalization condition is relocated
      OMINIT = OMEGA
      ALINIT = ALPHA
C
C
C---- set number of righthand sides
      NRHS = 2
      IF(LRE .EQ. 2) NRHS = 3
C
      CALL OS_INIT
C
C---- Newton iteration loop
      DO 100 ITER=1, ITMAX
C
        CALL OS_SETUP
        CALL OS_SOLVE
        CALL OS_UPDATE
C
        CALL OS_BCCHEK
C
CCC        call newpen(1)
CCC        do 66 i=1, n
CCC          utr(i) = real(f1(i))
CCC   66   continue
CCC        call urplot(n,y,utr)
C
        DELMAX = DFMAX
C
        IF(ITMAX.EQ.1) GO TO 101
C
c        IF(LRE.EQ.1) THEN
c         IF(LST.EQ.1) 
c     &    WRITE(*,7011) ITER,DFMAX,REAL(ALPHA),IMAG(ALPHA)
c 7011     FORMAT(1X,I2,'  max =', E11.4,'  a =', 2F10.6)
c         IF(LST.EQ.2)
c     &    WRITE(*,7012) ITER,DFMAX,REAL(OMEGA),IMAG(OMEGA)
c 7012     FORMAT(1X,I2,'  max =', E11.4,'  w =', 2F10.6)
c        ELSE
c         IF(LST.EQ.1)
c     &    WRITE(*,7021) ITER,DFMAX,REAL(ALPHA),IMAG(ALPHA),IMAG(IRE)
c 7021    FORMAT(1X,I2,'  max =', E11.4,'  a =', 2F10.6,'  Re =',E11.4)
c         IF(LST.EQ.2)
c     &    WRITE(*,7022) ITER,DFMAX,REAL(OMEGA),IMAG(OMEGA),IMAG(IRE)
c 7022    FORMAT(1X,I2,'  max =', E11.4,'  w =', 2F10.6,'  Re =',E11.4)
c        ENDIF
C
        IF(ISOL.NE.0 .AND. DFMAX .LT. EPS) GO TO 101
  100 CONTINUE
      WRITE(*,*) 'ORRS: Convergence failed.  Continuing ...'
C
  101 CONTINUE
C
C---- save variables for passing back to calling routine
      ALPHAR = REAL(ALPHA)
      ALPHAI = IMAG(ALPHA)
      OMEGAR = REAL(OMEGA)
      OMEGAI = IMAG(OMEGA)
      REI = IMAG(IRE)
C
      DO 200 I=1, N
        UTR(I) =  REAL(F1(I))
        UTI(I) =  IMAG(F1(I))
        VTR(I) =  REAL((0.0,-1.0)*ALPHA*F0(I))
        VTI(I) =  IMAG((0.0,-1.0)*ALPHA*F0(I))
        WTR(I) = -REAL(F2(I))
        WTI(I) = -IMAG(F2(I))
        CTR(I) = -REAL(F3(I))
        CTI(I) = -IMAG(F3(I))
  200 CONTINUE
C
      RETURN
      END ! ORRS


      SUBROUTINE OS_INIT
      INCLUDE 'ORRS.INC'
C
      DO I=1, N
        F0(I) = 0.
        F1(I) = 0.
        F2(I) = 0.
        F3(I) = 0.
      ENDDO
C
      ISOL = 0
C
      RETURN
      END


      SUBROUTINE OS_BCCHEK
      INCLUDE 'ORRS.INC'
      COMPLEX FFACT
C
      FWALL = CABS(F2(1))
      FEDGE = CABS(F2(N))
C
      IF(IBC .EQ. 1  .AND.  FEDGE .GT. 2.0*FWALL) THEN
       WRITE(*,*) 'Switching normalizing condition to edge'
       IBC = 2
       FFACT = FNORM/F2(N)
      ELSE IF(IBC .EQ. 2  .AND.  FWALL .GT. 2.0*FEDGE) THEN
       WRITE(*,*) 'Switching normalizing condition to wall'
       IBC = 1
       FFACT = FNORM/F2(1)
      ELSE
       RETURN
      ENDIF
C
      DO I=1, N
        F0(I) = F0(I)*FFACT
        F1(I) = F1(I)*FFACT
        F2(I) = F2(I)*FFACT
        F3(I) = F3(I)*FFACT
      ENDDO
C
      ISOL = 0
      ITMAX = MIN0( ITMAX + 1 , 20 )
      IF(LST.EQ.1) ALPHA = ALINIT
      IF(LST.EQ.2) OMEGA = OMINIT
C
      RETURN
      END ! OS_BCCHEK


      SUBROUTINE OS_SETUP
      INCLUDE 'ORRS.INC'
C------------------------------------------------------
C     Sets up 4x4 block-tridiagnonal system 
C     for Orr-Sommerfeld equation solution.
C
C     The perturbation stream function has the form:
C
C         P(x,y,t;a,w,R,U)  =  p(y) exp[i(ax - wt)]
C
C     The four equations set up are:
C
C         p' = q    2
C         q' = r + a p
C         r' = s                     2
C         s' = iR[(aU-w)r - aU"p] + a p
C
C     p = streamfunction  = F0
C     q = velocity        = F1
C     r = vorticity       = F2
C     s = dr/dy           = F3
C
C------------------------------------------------------
C
C---- zero out A,B,C blocks and righthand sides R
      DO I=1, N
        DO J=1, 4
          DO K=1, 4
            A(J,K,I) = (0.0,0.0)
            B(J,K,I) = (0.0,0.0)
            C(J,K,I) = (0.0,0.0)
          ENDDO
          DO K=1, NRMAX
            R(J,K,I) = (0.0,0.0)
          ENDDO
        ENDDO
      ENDDO
C
      I = 1
C
C---- set 1st wall BC
      R(1,1,I) = F0(I)
      A(1,1,I) = 1.0
C
      IF(IBC.EQ.1) THEN
C
C----- set normalizing condition in lieu of 2nd wall BC (enforced in OS_UPDATE)
       R(2,1,I) = F2(I) - FNORM
       A(2,3,I) = 1.0
C
      ELSE
C
C----- set 2nd wall BC
       R(2,1,I) = F1(I)
       A(2,2,I) = 1.0
C
      ENDIF
C
C---- set interior equations
      DO 50 I=1,N-1
C
        DY = Y(I+1) - Y(I)
        UAV = 0.5*(U(I+1) + U(I))
        UDD = UD(I+1) - UD(I)
C---------------------------------------------------------------
C
        R(1,1,I+1) = F0(I+1) - F0(I) - 0.5*DY*(F1(I+1)+F1(I))
        B(1,1,I+1) = -1.0
        A(1,1,I+1) =  1.0
        B(1,2,I+1) = -0.5*DY
        A(1,2,I+1) = -0.5*DY
C---------------------------------------------------------------
C
        R(2,1,I+1) = F1(I+1) - F1(I)
     &             - 0.5*DY*(   F2(I+1)+F2(I)
     &                         + (F0(I+1)+F0(I))*ALPHA**2 )
        IF(LST.EQ.1)
     &  R(2,2,I+1) = -0.5*DY * (F0(I+1)+F0(I)) * 2.0*ALPHA
        B(2,1,I+1) = -0.5*DY*ALPHA**2
        A(2,1,I+1) = -0.5*DY*ALPHA**2
        B(2,2,I+1) = -1.0
        A(2,2,I+1) =  1.0
        B(2,3,I+1) = -0.5*DY
        A(2,3,I+1) = -0.5*DY
C---------------------------------------------------------------
C
        R(3,1,I) = F2(I+1) - F2(I) - 0.5*DY*(F3(I+1)+F3(I))
        A(3,3,I) = -1.0
        C(3,3,I) =  1.0
        A(3,4,I) = -0.5*DY
        C(3,4,I) = -0.5*DY
C---------------------------------------------------------------
C
        R(4,1,I) = F3(I+1) - F3(I)
     &             - 0.5*DY* (F2(I+1)+F2(I)) * ALPHA**2
     &             - IRE*( (ALPHA*UAV-OMEGA)*0.5*DY*(F2(I+1)+F2(I))
     &                    - ALPHA*UDD*0.5*(F0(I+1)+F0(I)) )
        IF(LST.EQ.1)
     &  R(4,2,I) = - 0.5*DY* (F2(I+1)+F2(I)) * 2.0*ALPHA
     &             - IRE*(        UAV       *0.5*DY*(F2(I+1)+F2(I))
     &                    -       UDD*0.5*(F0(I+1)+F0(I)) )
        IF(LST.EQ.2)
     &  R(4,2,I) = - IRE*( (         -1.0  )*0.5*DY*(F2(I+1)+F2(I)) )
        R(4,3,I) = 
     &        -(0.0,1.0)*( (ALPHA*UAV-OMEGA)*0.5*DY*(F2(I+1)+F2(I))
     &                    - ALPHA*UDD*0.5*(F0(I+1)+F0(I)) )
        A(4,1,I) =                      IRE* ALPHA*UDD*0.5
        C(4,1,I) =                      IRE* ALPHA*UDD*0.5
        A(4,3,I) = -0.5*DY*ALPHA**2 - IRE*(ALPHA*UAV-OMEGA)*0.5*DY
        C(4,3,I) = -0.5*DY*ALPHA**2 - IRE*(ALPHA*UAV-OMEGA)*0.5*DY
        A(4,4,I) = -1.0
        C(4,4,I) =  1.0
C---------------------------------------------------------------
C
   50 CONTINUE
C
C---- set asymptotic regularity conditions at outer edge
C
      FACSQ  = ALPHA**2 + IRE*(ALPHA*U(N)-OMEGA)
      FAC    = CSQRT(FACSQ)
      IF(REAL(FACSQ) .LT. 0.0  .AND.  IMAG(FACSQ) .LT. 0.0) THEN
CCC       WRITE(*,*) 'ORRS: Overdamped mode.'
       FAC = -FAC
      ENDIF
      FAC_AL = (2.0*ALPHA + IRE*U(N)) * 0.5/FAC
      FAC_OM = (          - IRE     ) * 0.5/FAC
      FAC_RE = (0.0,1.0)*(ALPHA*U(N)-OMEGA) * 0.5/FAC
C
      IF(IBC.EQ.2) THEN
C
C----- set normalization condition in lieu of asymptotic regularity condition
       R(3,1,N) = F2(N) - FNORM
       A(3,3,N) = 1.0
C
      ELSE
C
       R(3,1,N) = (ALPHA + FAC   )*(F1(N) + F0(N)*ALPHA) + F2(N)
       IF(LST.EQ.1)
     & R(3,2,N) = (1.0   + FAC_AL)*(F1(N) + F0(N)*ALPHA)
     &          + (ALPHA + FAC   )*(        F0(N)      )
       IF(LST.EQ.2)
     & R(3,2,N) = (        FAC_OM)*(F1(N) + F0(N)*ALPHA)
       R(3,3,N) = (        FAC_RE)*(F1(N) + F0(N)*ALPHA)
       A(3,1,N) = (ALPHA + FAC   )*(              ALPHA)
       A(3,2,N) = (ALPHA + FAC   )
       A(3,3,N) = 1.0
C
      ENDIF
C
      R(4,1,N) = F3(N) + F2(N)*FAC
      IF(LST.EQ.1)
     &R(4,2,N) =         F2(N)*FAC_AL
      IF(LST.EQ.2)
     &R(4,2,N) =         F2(N)*FAC_OM
      R(4,3,N) =         F2(N)*FAC_RE
      A(4,3,N) =               FAC
      A(4,4,N) = 1.0
C
      RETURN
      END ! OS_SETUP


      SUBROUTINE OS_SOLVE
      INCLUDE 'ORRS.INC'
      COMPLEX PIVOT, TEMP
C---------------------------------------------------
C     4x4 complex tridiagonal block solver.
C     Customized for Orr-Sommerfeld equation system,
C     with certain entries assumed to be zero.
C     (Gives large CPU speedup).
C
C      Assumed initial structure for a block row:
C
C         p q r s   p q r s   p q r s
C        |* * # 0| |* * 0 0| |0 0 0 0|  <--  p' = q    2
C        |* * * 0| |* * * 0| |0 0 0 0|  <--  q' = r + a p
C        |# # # 0| |* * * *| |0 0 * *|  <--  r' = s                     2
C        |# # # 0| |* * * *| |* 0 * *|  <--  s' = iR[(au-w)r - au"p] + a p
C
C         B block   A block   C block
C
C        *  assumed nonzero in initial system
C        #  assumed zero in initial system, becoming nonzero due to fill-in
C        0  assumed zero always
C---------------------------------------------------
C
CCC** Backward sweep: Elimination of upper block diagonal (C's).
      DO 1 I=N, 1, -1
C
        IP = I+1
C
C------ don't eliminate Cn block because it doesn't exist
        IF(I.EQ.N) GO TO 12
C
C------ eliminate Ci block, thus modifying Ai and Ri blocks
        DO 111 L=1, 3
          K = 3
          A(K,L,I) = A(K,L,I)
     &             - C(K,3,I)*B(3,L,IP)
     &             - C(K,4,I)*B(4,L,IP)
          K = 4
          A(K,L,I) = A(K,L,I)
     &             - C(K,1,I)*B(1,L,IP)
     &             - C(K,3,I)*B(3,L,IP)
     &             - C(K,4,I)*B(4,L,IP)
  111   CONTINUE
        DO 112 L=1, NRHS
          K = 3
          R(K,L,I) = R(K,L,I)
     &             - C(K,3,I)*R(3,L,IP)
     &             - C(K,4,I)*R(4,L,IP)
          K = 4
          R(K,L,I) = R(K,L,I)
     &             - C(K,1,I)*R(1,L,IP)
     &             - C(K,3,I)*R(3,L,IP)
     &             - C(K,4,I)*R(4,L,IP)
  112   CONTINUE
C
C                                                              -1
CCC---- multiply Bi block and righthand side Ri vectors by (Ai)
C       using Gaussian elimination.
C
   12   CONTINUE
C        
        DO 13 KPIV=4, 2, -1
C
          KM1 = KPIV-1
C
          PIVOT = 1.0/A(KPIV,KPIV,I)
C
C-------- normalize pivot row
          DO 132 L=1, KM1
            A(KPIV,L,I) = A(KPIV,L,I)*PIVOT
  132     CONTINUE
C
          B(KPIV,1,I) = B(KPIV,1,I)*PIVOT
          B(KPIV,2,I) = B(KPIV,2,I)*PIVOT
          B(KPIV,3,I) = B(KPIV,3,I)*PIVOT
C
          DO 134 L=1, NRHS
            R(KPIV,L,I) = R(KPIV,L,I)*PIVOT
  134     CONTINUE
C
C-------- eliminate upper off-diagonal element in Ai block
          K = KM1
            TEMP = A(K,KPIV,I)
            DO 1351 L=KM1, 1, -1
              A(K,L,I) = A(K,L,I) - TEMP*A(KPIV,L,I)
 1351       CONTINUE
            B(K,1,I) = B(K,1,I) - TEMP*B(KPIV,1,I)
            B(K,2,I) = B(K,2,I) - TEMP*B(KPIV,2,I)
            B(K,3,I) = B(K,3,I) - TEMP*B(KPIV,3,I)
            DO 1352 L=1, NRHS
              R(K,L,I) = R(K,L,I) - TEMP*R(KPIV,L,I)
 1352       CONTINUE
C
   13   CONTINUE
C
C
C------ solve for first row
        PIVOT = 1.0/A(1,1,I)
        B(1,1,I) = B(1,1,I)*PIVOT
        B(1,2,I) = B(1,2,I)*PIVOT
        B(1,3,I) = B(1,3,I)*PIVOT
        DO 14 L=1, NRHS
          R(1,L,I) = R(1,L,I)*PIVOT
   14   CONTINUE
C
C------ back substitute (eliminate everything below diagonal in Ai block)
        DO 15 L=1, 3
          B(2,L,I) = B(2,L,I) - A(2,1,I)*B(1,L,I)
          B(3,L,I) = B(3,L,I) - A(3,1,I)*B(1,L,I)
     &                        - A(3,2,I)*B(2,L,I)
          B(4,L,I) = B(4,L,I) - A(4,1,I)*B(1,L,I)
     &                        - A(4,2,I)*B(2,L,I)
     &                        - A(4,3,I)*B(3,L,I)
   15   CONTINUE
C
        DO 16 L=1, NRHS
          R(2,L,I) = R(2,L,I) - A(2,1,I)*R(1,L,I)
          R(3,L,I) = R(3,L,I) - A(3,1,I)*R(1,L,I)
     &                        - A(3,2,I)*R(2,L,I)
          R(4,L,I) = R(4,L,I) - A(4,1,I)*R(1,L,I)
     &                        - A(4,2,I)*R(2,L,I)
     &                        - A(4,3,I)*R(3,L,I)
  16    CONTINUE
C
    1 CONTINUE
C
CCC** Forward sweep: Back substitution using lower block diagonal (Bi's).
      DO 2 I=2, N
        IM = I-1
        DO 21 L=1, NRHS
          DO 211 K=1, 4
            R(K,L,I) = R(K,L,I)
     &        - (  R(1,L,IM)*B(K,1,I)
     &           + R(2,L,IM)*B(K,2,I)
     &           + R(3,L,IM)*B(K,3,I) )
  211     CONTINUE
   21   CONTINUE
    2 CONTINUE
C
      RETURN
      END ! OS_SOLVE


      SUBROUTINE OS_UPDATE
      INCLUDE 'ORRS.INC'
      COMPLEX DF0,DF1,DF2,DF3
      COMPLEX DAW
      COMPLEX RES, RES_AL, RES_OM, RES_RE, RES_F0, RES_F1, RES_F2,
     &             RES_AW
C
      IF(ISOL.EQ.0) THEN
C
C----- no mode solution yet -- don't try to converge on eigenvalue
       DAW = (0.0,0.0)
       DRE = 0.0
C
      ELSE
C
C----- drive eigenvalue (alpha or omega) to satisfy dropped BC at wall or edge
       IF(IBC.EQ.1) THEN
C
C------ wall BC was dropped -- enforce it here
        I = 1
        DAW = (F1(I) - R(2,1,I)) / R(2,2,I)
        DRE = 0.0
C
       ELSE
C
C------ edge BC was dropped -- enforce it here
        RES    = (ALPHA + FAC   )*(F1(N) + F0(N)*ALPHA) + F2(N)
        RES_AL = (1.0   + FAC_AL)*(F1(N) + F0(N)*ALPHA)
     &         + (ALPHA + FAC   )*(        F0(N)      )
        RES_OM = (        FAC_OM)*(F1(N) + F0(N)*ALPHA)
        RES_RE = (        FAC_RE)*(F1(N) + F0(N)*ALPHA)
        RES_F0 = (ALPHA + FAC   )*(              ALPHA)
        RES_F1 = (ALPHA + FAC   )
        RES_F2 = 1.0
C
        IF(LST.EQ.1) RES_AW = RES_AL
        IF(LST.EQ.2) RES_AW = RES_OM
C
        DAW =-(RES   -RES_F0*R(1,1,N)-RES_F1*R(2,1,N)-RES_F2*R(3,1,N))
     &      / (RES_AW-RES_F0*R(1,2,N)-RES_F1*R(2,2,N)-RES_F2*R(3,2,N))
C
       ENDIF
C
      ENDIF
C
C---- set either alpha or omega change (spatial or temporal problem)
      IF(LST.EQ.1) THEN
       DALPHA = DAW
       DOMEGA = (0.0,0.0)
      ELSE
       DALPHA = (0.0,0.0)
       DOMEGA = DAW
      ENDIF
C
C
      RLX = 1.0
C
      DALF = REAL(DALPHA)/ABS(ALPHA)
      IF(RLX*DALF .LT. -.1) RLX = -.1/DALF
      IF(RLX*DALF .GT. 0.1) RLX = 0.1/DALF
C
      DALF = IMAG(DALPHA)/ABS(ALPHA)
      IF(RLX*DALF .LT. -.1) RLX = -.1/DALF
      IF(RLX*DALF .GT. 0.1) RLX = 0.1/DALF
C
      DOMF = REAL(DOMEGA)/ABS(OMEGA)
      IF(RLX*DOMF .LT. -.1) RLX = -.1/DOMF
      IF(RLX*DOMF .GT. 0.1) RLX = 0.1/DOMF
C
      DOMF = IMAG(DOMEGA)/ABS(OMEGA)
      IF(RLX*DOMF .LT. -.1) RLX = -.1/DOMF
      IF(RLX*DOMF .GT. 0.1) RLX = 0.1/DOMF
C
C      DREF = DRE / IMAG(IRE)
C      IF(RLX*DREF .LT. -.2) RLX = -.2/DREF
C      IF(RLX*DREF .GT. 0.3) RLX = 0.3/DREF
CC
C
C==== see if normalizing condition position needs to be changed
C
cC---- predicted wall and edge f" values at next iteration level
c      FWALL = CABS(F2(1) - R(3,1,1) - DAW*R(3,2,1) - DRE*R(3,3,1))
c      FEDGE = CABS(F2(N) - R(3,1,N) - DAW*R(3,2,N) - DRE*R(3,3,N))
cC
cC---- set flag to normalize whatever is bigger by factor of 2
cC
c      IF(IBC .EQ. 1  .AND.  FEDGE .GT. 2.0*FWALL) THEN
cC
c       WRITE(*,*) 'Switching normalizing condition to edge'
c       IBC = 2
c       ITMAX = MIN0( ITMAX+1 , 20 )
c       IF(LST.EQ.1) ALPHA = ALINIT
c       IF(LST.EQ.2) OMEGA = OMINIT
c       RETURN
cC
c      ELSE IF(IBC .EQ. 2  .AND.  FWALL .GT. 2.0*FEDGE) THEN
cC
c       WRITE(*,*) 'Switching normalizing condition to wall'
c       IBC = 1
c       ITMAX = MIN0( ITMAX+1 , 20 )
c       IF(LST.EQ.1) ALPHA = ALINIT
c       IF(LST.EQ.2) OMEGA = OMINIT
c       RETURN
cC
c      ENDIF
C
C
      DFMAX = 0.0
      DFRMS = 0.0
C
C---- perform Newton update on modes
      DO 50 I=1, N
        DF0 = -R(1,1,I) - DAW*R(1,2,I) - DRE*R(1,3,I)
        DF1 = -R(2,1,I) - DAW*R(2,2,I) - DRE*R(2,3,I)
        DF2 = -R(3,1,I) - DAW*R(3,2,I) - DRE*R(3,3,I)
        DF3 = -R(4,1,I) - DAW*R(4,2,I) - DRE*R(4,3,I)
C
        F0(I) = F0(I) + RLX*DF0
        F1(I) = F1(I) + RLX*DF1
        F2(I) = F2(I) + RLX*DF2
        F3(I) = F3(I) + RLX*DF3
C
        D0SQ = (REAL(DF0)**2 + IMAG(DF0)**2)
        D1SQ = (REAL(DF1)**2 + IMAG(DF1)**2)
        D2SQ = (REAL(DF2)**2 + IMAG(DF2)**2)
        D3SQ = (REAL(DF3)**2 + IMAG(DF3)**2)
C
C       IF(D0SQ .GT. DFMAX) THEN
C        KVMAX = 0
C        IVMAX = I
C        DFMAX = D0SQ
C       ENDIF
C
C       IF(D1SQ .GT. DFMAX) THEN
C        KVMAX = 1
C        IVMAX = I
C        DFMAX = D1SQ
C       ENDIF
C
C       IF(D2SQ .GT. DFMAX) THEN
C        KVMAX = 2
C        IVMAX = I
C        DFMAX = D2SQ
C       ENDIF
C
C       IF(D3SQ .GT. DFMAX) THEN
C        KVMAX = 3
C        IVMAX = I
C        DFMAX = D3SQ
C       ENDIF
C
        DFMAX = MAX( DFMAX , D0SQ , D1SQ , D2SQ , D3SQ )
        DFRMS =      DFRMS + D0SQ + D1SQ + D2SQ + D3SQ
   50 CONTINUE
C
      DFMAX = SQRT( DFMAX )
      DFRMS = SQRT( DFRMS / (4.0*FLOAT(N)) )
C
C---- perform Newton update on eigenvalues
      ALPHA = ALPHA + RLX*DALPHA
      OMEGA = OMEGA + RLX*DOMEGA
      IRE   = IRE   + RLX*CMPLX(0.0,DRE)
C
C---- modes are now available 
      ISOL = 1
C
      RETURN
      END ! OS_UPDATE

