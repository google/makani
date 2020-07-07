
      SUBROUTINE OSPRES(NI,YI,UI, ALPHAR,ALPHAI, VTR,VTI,
     &                  PTR,PTI )
      DIMENSION YI(NI), UI(NI)
      DIMENSION VTR(NI), VTI(NI)
      DIMENSION PTR(NI), PTI(NI)
C---------------------------------------------------------------------
C     Routine for calculating the Orr-Sommerfeld pressure profile.
C
C     Input:
C     ------
C      NI      total number of points in profiles
C      YI      normal BL coordinate array
C      UI      mean flow u(y) profile
C      ALPHAR  real  part of complex wavenumber
C      ALPHAI  imag. part of complex wavenumber
C      VTR     real  part of perturbation y-velocity profile
C      VTI     imag. part of perturbation y-velocity profile
C
C     Output:
C     -------
C      PTR     real  part of perturbation pressure profile
C      PTI     imag. part of perturbation pressure profile
C---------------------------------------------------------------------
C
      INCLUDE 'OSPRES.INC'
C
C---- convergence tolerance
      DATA EPS / 1.0E-4 /
C
      IF(NI.GT.NMAX) STOP 'OSPRES: Array overflow.'
C
      N = NI
      DO 5 I=1, N
        Y(I) = YI(I)
        U(I) = UI(I)
        VT(I) = CMPLX( VTR(I) , VTI(I) )
    5 CONTINUE
C
      ALPHA = CMPLX(ALPHAR,ALPHAI)
C
C---- set number of righthand sides
      NRHS = 1
C
      DO I=1, N
        F0(I) = 0.
        F1(I) = 0.
      ENDDO
      ISOL = 0
C
      CALL SETUP_P
      CALL SOLVE_P
      CALL UPDATE_P
C
      DO 200 I=1, N
        PTR(I) = REAL(F0(I))
        PTI(I) = IMAG(F0(I))
  200 CONTINUE
C
      RETURN
      END ! OSPRES


      SUBROUTINE SETUP_P
      INCLUDE 'OSPRES.INC'
      COMPLEX VTA
C
C---- zero out A,B,C blocks and righthand sides R
      DO 20 I=1, N
        DO 201 J=1, 2
          DO 2001 K=1, 2
            A(J,K,I) = (0.0,0.0)
            B(J,K,I) = (0.0,0.0)
            C(J,K,I) = (0.0,0.0)
 2001     CONTINUE
          DO 2002 K=1, NRMAX
            R(J,K,I) = (0.0,0.0)
 2002     CONTINUE
  201   CONTINUE
   20 CONTINUE
C
      I = 1
C
C---- set 1st wall BC
      R(2,1,I) = F1(I)
      A(2,2,I) = 1.0
C
C---- set interior equations
      DO 50 I=1,N-1
C
        DY =  Y(I+1) - Y(I)
        DU =  U(I+1) - U(I)
C
C---------------------------------------------------------------
C
        R(1,1,I) = F0(I+1) - F0(I) - 0.5*DY*(F1(I+1)+F1(I))
        A(1,1,I) = -1.0
        C(1,1,I) =  1.0
        A(1,2,I) = -0.5*DY
        C(1,2,I) = -0.5*DY
C---------------------------------------------------------------
C
        R(2,1,I+1) = F1(I+1) - F1(I) - 0.5*DY*(F0(I+1)+F0(I))*ALPHA**2
     &             + (0.0,1.0)*ALPHA*DU*(VT(I+1) + VT(I))
        B(2,1,I+1) = -0.5*DY*ALPHA**2
        A(2,1,I+1) = -0.5*DY*ALPHA**2
        B(2,2,I+1) = -1.0
        A(2,2,I+1) =  1.0
C---------------------------------------------------------------
C
   50 CONTINUE
C
C---- set asymptotic regularity conditions at outer edge
C
      R(1,1,N) = F1(N) + F0(N)*ALPHA
      A(1,1,N) =               ALPHA
      A(1,2,N) = 1.0
C
      RETURN
      END ! SETUP


      SUBROUTINE SOLVE_P
      INCLUDE 'OSPRES.INC'
      COMPLEX PIVOT, TEMP
C---------------------------------------------------
C     2x2 complex tridiagonal block solver.
C---------------------------------------------------
C
CCC** Forward sweep: Elimination of lower block diagonal (B's).
      DO 1 I=1, N
C
        IM = I-1
C
C------ don't eliminate B1 block because it doesn't exist
        IF(I.EQ.1) GO TO 12
C
C------ eliminate Ci block, thus modifying Ai and Ri blocks
        DO 111 L=1, 2
          K = 1
          A(K,L,I) = A(K,L,I)
     &             - B(K,1,I)*C(1,L,IM)
     &             - B(K,2,I)*C(2,L,IM)
          K = 2
          A(K,L,I) = A(K,L,I)
     &             - B(K,1,I)*C(1,L,IM)
     &             - B(K,2,I)*C(2,L,IM)
  111   CONTINUE
        DO 112 L=1, NRHS
          K = 1
          R(K,L,I) = R(K,L,I)
     &             - B(K,1,I)*R(1,L,IM)
     &             - B(K,2,I)*R(2,L,IM)
          K = 2
          R(K,L,I) = R(K,L,I)
     &             - B(K,1,I)*R(1,L,IM)
     &             - B(K,2,I)*R(2,L,IM)
  112   CONTINUE
C
C                                                              -1
CCC---- multiply Ci block and righthand side Ri vectors by (Ai)
C       using Gaussian elimination.
C
   12   CONTINUE
C        
        DO 13 KPIV=1, 2
C
          KP1 = KPIV+1
C
          PIVOT = 1.0/A(KPIV,KPIV,I)
C
C-------- normalize pivot row
          DO 132 L=KP1, 2
            A(KPIV,L,I) = A(KPIV,L,I)*PIVOT
  132     CONTINUE
C
          C(KPIV,1,I) = C(KPIV,1,I)*PIVOT
          C(KPIV,2,I) = C(KPIV,2,I)*PIVOT
C
          DO 134 L=1, NRHS
            R(KPIV,L,I) = R(KPIV,L,I)*PIVOT
  134     CONTINUE
C
C-------- eliminate lower off-diagonal elements in Ai block
          DO 135 K=KP1, 2
            TEMP = A(K,KPIV,I)
            DO 1351 L=KP1, 2
              A(K,L,I) = A(K,L,I) - TEMP*A(KPIV,L,I)
 1351       CONTINUE
            C(K,1,I) = C(K,1,I) - TEMP*C(KPIV,1,I)
            C(K,2,I) = C(K,2,I) - TEMP*C(KPIV,2,I)
            DO 1352 L=1, NRHS
              R(K,L,I) = R(K,L,I) - TEMP*R(KPIV,L,I)
 1352       CONTINUE
 135      CONTINUE
C
   13   CONTINUE
C
C------ back substitute everything
        DO 15 KPIV=1, 1, -1
          KP1 = KPIV+1
          DO 151 K=KP1, 2
            C(KPIV,1,I) = C(KPIV,1,I) - A(KPIV,K,I)*C(K,1,I)
            C(KPIV,2,I) = C(KPIV,2,I) - A(KPIV,K,I)*C(K,2,I)
            DO 1511 L=1, NRHS
              R(KPIV,L,I) = R(KPIV,L,I) - A(KPIV,K,I)*R(K,L,I)
 1511       CONTINUE
  151     CONTINUE
   15   CONTINUE
C
    1 CONTINUE
C
CCC** Backward sweep: Back substitution using upper block diagonal (Ci's).
      DO 2 I=N-1, 1, -1
        IP = I+1
        DO 21 L=1, NRHS
          DO 211 K=1, 2
            R(K,L,I) = R(K,L,I)
     &              - (R(1,L,IP)*C(K,1,I) + R(2,L,IP)*C(K,2,I))
  211     CONTINUE
   21   CONTINUE
    2 CONTINUE
C
      RETURN
      END ! SOLVE


      SUBROUTINE UPDATE_P
      INCLUDE 'OSPRES.INC'
      COMPLEX DF0,DF1
C
      RLX = 1.0
C
C---- perform Newton update on modes
      DO 50 I=1, N
        DF0 = -R(1,1,I)
        DF1 = -R(2,1,I)
C
        F0(I) = F0(I) + RLX*DF0
        F1(I) = F1(I) + RLX*DF1
C
   50 CONTINUE
C
      RETURN
      END ! UPDATE

