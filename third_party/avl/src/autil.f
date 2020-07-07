C***********************************************************************
C    Module:  autil.f
C 
C    Copyright (C) 2002 Mark Drela, Harold Youngren
C 
C    This program is free software; you can redistribute it and/or modify
C    it under the terms of the GNU General Public License as published by
C    the Free Software Foundation; either version 2 of the License, or
C    (at your option) any later version.
C
C    This program is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
C    GNU General Public License for more details.
C
C    You should have received a copy of the GNU General Public License
C    along with this program; if not, write to the Free Software
C    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C***********************************************************************

      SUBROUTINE M3INV(A,AINV)
      DIMENSION A(3,3), AINV(3,3)
C----------------------------------------------
C     Calculates inverse of 3x3 matrix with 
C     special treatment...
C
C       If a diagonal element is exactly zero, 
C       then it is interpreted as infinity.
C----------------------------------------------
      REAL T(3,6)
C
C---- set augmented matrix
      DO K=1, 3
        T(K,1) = A(K,1)
        T(K,2) = A(K,2)
        T(K,3) = A(K,3)
        T(K,4) = 0.
        T(K,5) = 0.
        T(K,6) = 0.
      ENDDO
      T(1,4) = 1.0
      T(2,5) = 1.0
      T(3,6) = 1.0
C
      DO N=1, 3
        PIVOT = T(N,N)
C
C------ normalize pivot row...
        IF(PIVOT .EQ. 0.0) THEN
C-------- interpret pivot as infinity
          DO L=N+1, 6
            T(N,L) = 0.0
          ENDDO
        ELSE
C-------- interpret pivot as-is
          DO L=N+1, 6
            T(N,L) = T(N,L)/PIVOT
          ENDDO
        ENDIF
C
C------ eliminate pivot column
        DO K=N+1, 3
          TEL = T(K,N)
          DO L=N+1, 6
            T(K,L) = T(K,L) - TEL*T(N,L)
          ENDDO
        ENDDO
      ENDDO
C
C---- back-substitute
      DO N=3, 2, -1
        DO K=N-1, 1, -1
          TEL = T(K,N)
          DO L=4, 6
            T(K,L) = T(K,L) - TEL*T(N,L)
          ENDDO
        ENDDO
      ENDDO
C
      DO K=1, 3
        AINV(K,1) = T(K,4)
        AINV(K,2) = T(K,5)
        AINV(K,3) = T(K,6)
      ENDDO
C
      RETURN
      END ! M3INV



      SUBROUTINE RATEKI3(A, R,R_A)
      DIMENSION A(3)
      DIMENSION R(3,3), R_A(3,3,3)
C-----------------------------------------------------
C                                    t
C     Calculates inverse of tensor  T K
C
C                            t
C     d{phi theta psi}  =  [T K] d{p q r}
C
C-----------------------------------------------------
      C1 = COS(A(1))
      C2 = COS(A(2))
C
      S1 = SIN(A(1))
C
      T2 = TAN(A(2))
C
      R(1,1)     = -1.0
      R(2,1)     = 0.
      R(3,1)     = 0.
C
      R(1,2)     =  S1*T2
      R(2,2)     =  C1
      R(3,2)     =  S1/C2
C
      R(1,3)     = -C1*T2
      R(2,3)     =  S1
      R(3,3)     = -C1/C2
C
      R_A(1,1,1) = 0.
      R_A(2,1,1) = 0.
      R_A(3,1,1) = 0.
C
      R_A(1,2,1) =  C1*T2
      R_A(2,2,1) = -S1
      R_A(3,2,1) =  C1/C2
C
      R_A(1,3,1) =  S1*T2
      R_A(2,3,1) =  C1
      R_A(3,3,1) =  S1/C2
C
      R_A(1,1,2) = 0.
      R_A(2,1,2) = 0.
      R_A(3,1,2) = 0.
C
      R_A(1,2,2) =  S1/C2**2
      R_A(2,2,2) = 0.
      R_A(3,2,2) =  S1*T2/C2
C
      R_A(1,3,2) = -C1/C2**2
      R_A(2,3,2) = 0.
      R_A(3,3,2) = -C1*T2/C2
C
      R_A(1,1,3) = 0.
      R_A(2,1,3) = 0.
      R_A(3,1,3) = 0.
C
      R_A(1,2,3) = 0.
      R_A(2,2,3) = 0.
      R_A(3,2,3) = 0.
C
      R_A(1,3,3) = 0.
      R_A(2,3,3) = 0.
      R_A(3,3,3) = 0.
C
      RETURN
      END ! RATEKI3



      SUBROUTINE ROTENS3(A,T,T_A)
      DIMENSION A(3), T(3,3), T_A(3,3,3)
C------------------------------------------
C     Calculates net rotation tensor T(..) 
C     from the three rotation angles A(.).
C
C     The order and sense of rotation from
C     X axes to X' axes is
C
C      X'  =   A3 [ -A2 [ A1 [ X ] ] ]
C
C      X'  =  T X
C------------------------------------------
C
      C1 = COS(A(1))   !  Phi
      C2 = COS(A(2))   !  Theta
      C3 = COS(A(3))   !  Psi
C
      S1 = SIN(A(1))
      S2 = SIN(A(2))
      S3 = SIN(A(3))
C
      T(1,1) =     C2*C3
      T(2,1) =    -C2*S3
      T(3,1) =    -S2
C
      T(1,2) = -S1*S2*C3 + C1*S3
      T(2,2) =  S1*S2*S3 + C1*C3
      T(3,2) = -S1*C2
C
      T(1,3) =  C1*S2*C3 + S1*S3
      T(2,3) = -C1*S2*S3 + S1*C3
      T(3,3) =  C1*C2
C
C
      T_A(1,1,1) = 0.
      T_A(2,1,1) = 0.
      T_A(3,1,1) = 0.
C
      T_A(1,2,1) = -C1*S2*C3 - S1*S3
      T_A(2,2,1) =  C1*S2*S3 - S1*C3
      T_A(3,2,1) = -C1*C2
C
      T_A(1,3,1) = -S1*S2*C3 + C1*S3
      T_A(2,3,1) =  S1*S2*S3 + C1*C3
      T_A(3,3,1) = -S1*C2
C
C
      T_A(1,1,2) =    -S2*C3
      T_A(2,1,2) =     S2*S3
      T_A(3,1,2) =    -C2
C
      T_A(1,2,2) = -S1*C2*C3
      T_A(2,2,2) =  S1*C2*S3
      T_A(3,2,2) =  S1*S2
C
      T_A(1,3,2) =  C1*C2*C3
      T_A(2,3,2) = -C1*C2*S3
      T_A(3,3,2) = -C1*S2
C
C
      T_A(1,1,3) =    -C2*S3
      T_A(2,1,3) =    -C2*C3
      T_A(3,1,3) = 0.
C
      T_A(1,2,3) =  S1*S2*S3 + C1*C3
      T_A(2,2,3) =  S1*S2*C3 - C1*S3
      T_A(3,2,3) = 0.
C
      T_A(1,3,3) = -C1*S2*S3 + S1*C3
      T_A(2,3,3) = -C1*S2*C3 - S1*S3
      T_A(3,3,3) = 0.
C
C
      RETURN
      END ! ROTENS3



