C***********************************************************************
C    Module:  aic.f
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


      SUBROUTINE VVOR(BETM,IYSYM,YSYM,IZSYM,ZSYM,VRCORE,
     &     NV,RV1,RV2,NSURFV,CHORDV,
     &     NC,RC ,    NSURFC,LVTEST,
     &     WC_GAM,NCDIM)
C--------------------------------------------------------------------
C     Calculates the velocity influence matrix for a collection 
C     of horseshoe vortices and control points
C     
C Input
C -----
C     BETM      SQRT(1-MACH*MACH)
C     IYSYM     Plane of symmetry XZ 
C                = 0 no symmetry
C                = 1 regular symmetry
C                =-1 free-surface symmetry
C     YSYM      Y coordinate of symmetry plane
C     IZSYM     Second plane of symmetry XY 
C                = 0 no second plane
C                = 1 regular symmetry
C                =-1 free-surface symmetry
C     ZSYM      Z coordinate of symmetry plane
C
C     VRCORE    vortex-line core radius / max(semichord,vortex width)
C
C     NV        number of vortices
C     RV1(3,v)  coordinates of endpoint #1 of the vortices
C     RV2(3,v)  coordinates of endpoint #2 of the vortices
C     NSURFV(v) index of surface containing h.v.
C     CHORDV(v) chord of strip containing h.v.
C
C     NC        number of control points
C     RC(3,c)   coordinates of the control points
C     NSURFC(c) index of surface containing c.p.
C     LVTEST    T if core-radius test is to be applied
C
C     NCDIM     declared size of WC_GAM matrix
C     
C Output
C ------
C     WC_GAM(3..)   Induced-velocity/gamma influence matrix
C     
C--------------------------------------------------------------------
      REAL RV1(3,NV),
     &     RV2(3,NV),
     &     CHORDV(NV)
      REAL RC(3,NC),
     &     WC_GAM(3,NCDIM,*)
      INTEGER NSURFV(NV), NSURFC(NC)
      LOGICAL LVTEST
C     
      LOGICAL LBOUND
C     
C     
      FYSYM = FLOAT(IYSYM)
      FZSYM = FLOAT(IZSYM)
C     
C...  Nested pair of loops to calculate the normalwash influence matrix
C     the outer loop runs over all the control points
C     the inner loop runs over all the vortex elements 
C     
      DO 200 I = 1, NC
C...  Control point location
         X = RC(1,I)
         Y = RC(2,I)
         Z = RC(3,I)
C     
         U = 0.
         V = 0.
         W = 0.
C     
         DO 100 J = 1, NV
C--------- set vortex core
            DSYZ = SQRT(  (RV2(2,J)-RV1(2,J))**2
     &                  + (RV2(3,J)-RV1(3,J))**2 )
            IF(NSURFC(I) .EQ. NSURFV(J)) THEN
             RCORE = 0.0001*DSYZ
            ELSE
             RCORE = MAX( VRCORE*CHORDV(J) , 2.0*VRCORE*DSYZ )
cc             RCORE = VRCORE*DSYZ
            ENDIF
C     
            UI = 0.0
            VI = 0.0
            WI = 0.0
C     
            YOFF = 2.0* YSYM
            ZOFF = 2.0* ZSYM
CCC   ZOFF = 2.0*(ZSYM + ALFA*0.5*(RV1(1,J)+RV2(1,J)) )
C     
C...  Calculate the influence of the REAL vortex

            LBOUND = .NOT.(LVTEST .AND. I.EQ.J)
            CALL VORVELC(X,Y,Z,LBOUND,
     &           RV1(1,J),RV1(2,J),RV1(3,J),
     &           RV2(1,J),RV2(2,J),RV2(3,J),
     &           BETM,U,V,W,RCORE)
C     
            IF(IYSYM.NE.0) THEN
C...  Calculate the influence of the y-IMAGE vortex
               LBOUND = .TRUE.
C...  For sym/asym matrices check for vortex midpoints of image vortices
               IF(IYSYM.EQ.1) THEN
                  XAVE =        0.5*(RV1(1,J)+RV2(1,J))
                  YAVE = YOFF - 0.5*(RV1(2,J)+RV2(2,J))
                  ZAVE =        0.5*(RV1(3,J)+RV2(3,J))
                  IF(X.EQ.XAVE .AND. 
     &               Y.EQ.YAVE .AND.
     &               Z.EQ.ZAVE       ) LBOUND = .FALSE.
ccc   IF(.NOT.LBOUND) write(*,*) 'POS self vortex i,j ',i,j
               ENDIF

               CALL VORVELC(X,Y,Z,LBOUND,
     &              RV2(1,J),YOFF-RV2(2,J),RV2(3,J),
     &              RV1(1,J),YOFF-RV1(2,J),RV1(3,J),
     &              BETM,UI,VI,WI,RCORE)

c               CALL VORVEL(X,Y,Z,LBOUND,
c     &              RV2(1,J),YOFF-RV2(2,J),RV2(3,J),
c     &              RV1(1,J),YOFF-RV1(2,J),RV1(3,J),
c     &              BETM,UI,VI,WI)

               UI = UI*FYSYM
               VI = VI*FYSYM
               WI = WI*FYSYM 
            ENDIF
C     
            IF(IZSYM.NE.0) THEN
C...  Calculate the influence of the z-IMAGE vortex
               LBOUND = .TRUE.
               CALL VORVELC(X,Y,Z,LBOUND,
     &              RV2(1,J),RV2(2,J),ZOFF-RV2(3,J),
     &              RV1(1,J),RV1(2,J),ZOFF-RV1(3,J),
     &              BETM,UII,VII,WII,RCORE)
               U = U + UII*FZSYM
               V = V + VII*FZSYM
               W = W + WII*FZSYM
C     
C...  Calculate the influence of the y,z-IMAGE vortex
               IF(IYSYM.NE.0) THEN
                  LBOUND = .TRUE.
                  CALL VORVELC(X,Y,Z,LBOUND,
     &                 RV1(1,J),YOFF-RV1(2,J),ZOFF-RV1(3,J),
     &                 RV2(1,J),YOFF-RV2(2,J),ZOFF-RV2(3,J),
     &                 BETM,UII,VII,WII,RCORE)
C     
                  UI = UI + UII*FYSYM*FZSYM
                  VI = VI + VII*FYSYM*FZSYM
                  WI = WI + WII*FYSYM*FZSYM
               ENDIF
            ENDIF
C     
            US = U + UI
            VS = V + VI
            WS = W + WI
C     
            WC_GAM(1,I,J) = US
            WC_GAM(2,I,J) = VS
            WC_GAM(3,I,J) = WS
C     
 100     CONTINUE

 200  CONTINUE
C     
      RETURN
      END



      SUBROUTINE VSRD(BETM,IYSYM,YSYM,IZSYM,ZSYM,SRCORE,
     &                NBODY,LFRST,NLDIM,
     &                NL,RL,RADL,
     &                NU,SRC_U,DBL_U,
     &                NC,RC,
     &                WC_U,NCDIM)
C--------------------------------------------------------------------
C     Calculates the velocity influence matrix for a collection 
C     of source+doublet lines
C     
C Input
C -----
C       BETM     SQRT(1-MACH*MACH)
C       IYSYM    Plane of symmetry XZ 
C                 = 0 no symmetry
C                 = 1 regular symmetry
C                 =-1 free-surface symmetry
C       YSYM     Y coordinate of symmetry plane
C       IZSYM    Second plane of symmetry XY 
C                 = 0 no second plane
C                 = 1 regular symmetry
C                 =-1 free-surface symmetry
C       ZSYM     Z coordinate of symmetry plane
C
C       SRCORE   source-line core radius / body radius
C
C       NBODY      number of bodies
C       LFRST(b)   index of first node in body b
C       NLDIM      size of SRC_U, DBL_U matrices
C       NL(b)      number of source-line nodes in each body
C       RL(3,b)    source-line node
C       RADL(b)    body radius at node
C
C       NU         number of apparent-freestream components
C       SRC_U(u)   source  strength per unit freestream component
C       DBL_U(3,u) doublet strength per unit freestream component
C
C       NC        number of control points
C       RC(3,c)   control point node where velocity is evaluated
C
C       NCDIM      size of WC matrix
C          
C Output
C ------
C       WC_U(3,c,u)  velocity per unit freestream
C
C--------------------------------------------------------------------
      INTEGER LFRST(*),NL(*)
      REAL RL(3,*), RADL(*)
      REAL SRC_U(NLDIM,*), DBL_U(3,NLDIM,*),
     &     RC(3,NCDIM),
     &     WC_U(3,NCDIM,*)
C
C
      REAL VSRC(3), VDBL(3,3)
      DATA PI / 3.14159265 /
C
      FYSYM = FLOAT(IYSYM)
      FZSYM = FLOAT(IZSYM)
C
      YOFF = 2.0* YSYM
      ZOFF = 2.0* ZSYM
CCC   ZOFF = 2.0*(ZSYM + ALFA*0.5*(RV1(1,J)+RV2(1,J)) )
C
      DO I = 1, NC
        DO IU = 1, NU
          WC_U(1,I,IU) = 0.
          WC_U(2,I,IU) = 0.
          WC_U(3,I,IU) = 0.
        ENDDO
      ENDDO
C
C
      DO 10 IBODY = 1, NBODY
        DO 105 ILSEG = 1, NL(IBODY)-1
          L1 = LFRST(IBODY) + ILSEG - 1
          L2 = LFRST(IBODY) + ILSEG
C
          L = L1
C
          RAVG = SQRT(0.5*(RADL(L2)**2 + RADL(L1)**2))
          RCORE = SRCORE*RAVG
C
          DO I = 1, NC
C------------------------------------------------------------
C---------- influence of real segment
            CALL SRDVELC(RC(1,I) ,RC(2,I) ,RC(3,I) ,
     &                   RL(1,L1),RL(2,L1),RL(3,L1),
     &                   RL(1,L2),RL(2,L2),RL(3,L2),
     &                   BETM,RCORE,
     &                   VSRC,VDBL  )
            DO IU = 1, NU
              DO K = 1, 3
                WC_U(K,I,IU) = WC_U(K,I,IU)
     &               + VSRC(K)*SRC_U(L,IU)
     &               + VDBL(K,1)*DBL_U(1,L,IU)
     &               + VDBL(K,2)*DBL_U(2,L,IU)
     &               + VDBL(K,3)*DBL_U(3,L,IU)
              ENDDO
            ENDDO
C
C------------------------------------------------------------
            IF (IYSYM.NE.0) THEN
C----------- influence of y-image
             CALL SRDVELC(RC(1,I) ,     RC(2,I) ,RC(3,I) ,
     &                    RL(1,L1),YOFF-RL(2,L1),RL(3,L1),
     &                    RL(1,L2),YOFF-RL(2,L2),RL(3,L2),
     &                    BETM,RCORE,
     &                    VSRC,VDBL  )
             DO IU = 1, NU
               DO K = 1, 3
                 WC_U(K,I,IU) = WC_U(K,I,IU)
     &                + VSRC(K)*SRC_U(L,IU)
     &                + VDBL(K,1)*DBL_U(1,L,IU)
     &                - VDBL(K,2)*DBL_U(2,L,IU)
     &                + VDBL(K,3)*DBL_U(3,L,IU)
               ENDDO
             ENDDO
            ENDIF
C
C------------------------------------------------------------
            IF (IZSYM.NE.0) THEN
C----------- influence of z-image
             CALL SRDVELC(RC(1,I) ,RC(2,I) ,     RC(3,I) ,
     &                    RL(1,L1),RL(2,L1),ZOFF-RL(3,L1),
     &                    RL(1,L2),RL(2,L2),ZOFF-RL(3,L2),
     &                    BETM,RCORE,
     &                    VSRC,VDBL  )
             DO IU = 1, NU
               DO K = 1, 3
                 WC_U(K,I,IU) = WC_U(K,I,IU)
     &                + VSRC(K)*SRC_U(L,IU)
     &                + VDBL(K,1)*DBL_U(1,L,IU)
     &                + VDBL(K,2)*DBL_U(2,L,IU)
     &                - VDBL(K,3)*DBL_U(3,L,IU)
               ENDDO
             ENDDO

C------------------------------------------------------------
             IF (IYSYM.NE.0) THEN
C------------ influence of z-image
              CALL SRDVELC(RC(1,I) ,     RC(2,I) ,     RC(3,I) ,
     &                     RL(1,L1),YOFF-RL(2,L1),ZOFF-RL(3,L1),
     &                     RL(1,L2),YOFF-RL(2,L2),ZOFF-RL(3,L2),
     &                     BETM,RCORE,
     &                     VSRC,VDBL  )
              DO IU = 1, NU
                DO K = 1, 3
                  WC_U(K,I,IU) = WC_U(K,I,IU)
     &                 + VSRC(K)*SRC_U(L,IU)
     &                 + VDBL(K,1)*DBL_U(1,L,IU)
     &                 - VDBL(K,2)*DBL_U(2,L,IU)
     &                 - VDBL(K,3)*DBL_U(3,L,IU)
                ENDDO
              ENDDO
             ENDIF
            ENDIF
C
          ENDDO
C
 105    CONTINUE
 10   CONTINUE
C
      RETURN
      END ! VSRD



      SUBROUTINE SRDSET(BETM,XYZREF,
     &                  NBODY,LFRST,NLDIM,
     &                  NL,RL,RADL,
     &                  SRC_U,DBL_U )
C----------------------------------------------------------
C     Sets strengths of source+doublet line segments
C     for each unit freestream and rotation component
C
C Input
C -----
C       BETM     SQRT(1-MACH*MACH)
C
C       NBODY      number of bodies
C       LFRST(b)   index of first node in body b
C       NLDIM      size of SRC_U, DBL_U matrices
C       NL(b)      number of source-line nodes in each body
C       RL(3,b)    source-line node
C       RADL(b)    body radius at node
C
C Output
C ------
C       SRC_U(u)   source  strength per unit freestream component
C       DBL_U(3,u) doublet strength per unit freestream component
C
C----------------------------------------------------------
      REAL XYZREF(3)
      REAL RL(3,*), RADL(*)
      INTEGER LFRST(*),NL(*)
      REAL SRC_U(NLDIM,*), DBL_U(3,NLDIM,*)
C
C
      REAL DRL(3), VSRC(3), VDBL(3,3)
      REAL ESL(3), UN(3)
      REAL WROT(3), UREL(3), RLREF(3)
C
      DATA PI / 3.14159265 /
C
      DO 10 IBODY = 1, NBODY
        DO 105 ILSEG = 1, NL(IBODY)-1
          L1 = LFRST(IBODY) + ILSEG - 1
          L2 = LFRST(IBODY) + ILSEG
C
          L = L1
C
          DRL(1) = (RL(1,L2) - RL(1,L1))/BETM
          DRL(2) =  RL(2,L2) - RL(2,L1)
          DRL(3) =  RL(3,L2) - RL(3,L1)
          DRLMAG = SQRT(DRL(1)**2 + DRL(2)**2 + DRL(3)**2)
          IF(DRLMAG.EQ.0.0) THEN
           DRLMI = 0.0
          ELSE
           DRLMI = 1.0/DRLMAG
          ENDIF
C
C-------- unit vector along line segment
          ESL(1) = DRL(1) * DRLMI
          ESL(2) = DRL(2) * DRLMI
          ESL(3) = DRL(3) * DRLMI
C
          ADEL = PI *     (RADL(L2)**2 - RADL(L1)**2)
          AAVG = PI * 0.5*(RADL(L2)**2 + RADL(L1)**2)
C
          RLREF(1) = 0.5*(RL(1,L2)+RL(1,L1)) - XYZREF(1)
          RLREF(2) = 0.5*(RL(2,L2)+RL(2,L1)) - XYZREF(2)
          RLREF(3) = 0.5*(RL(3,L2)+RL(3,L1)) - XYZREF(3)
C
C-------- go over freestream velocity and rotation components
          DO IU = 1, 6
            UREL(1) = 0.
            UREL(2) = 0.
            UREL(3) = 0.
            WROT(1) = 0.
            WROT(2) = 0.
            WROT(3) = 0.
C
            IF  (IU.LE.3) THEN
             UREL(IU) = 1.0
            ELSE
             WROT(IU-3) = 1.0
             CALL CROSS(RLREF,WROT,UREL)
            ENDIF
            UREL(1) = UREL(1)/BETM
C
C---------- U.es
            US = UREL(1)*ESL(1) + UREL(2)*ESL(2) + UREL(3)*ESL(3)
C
C---------- velocity projected on normal plane = U - (U.es) es
            UN(1) = UREL(1) - US*ESL(1)
            UN(2) = UREL(2) - US*ESL(2)
            UN(3) = UREL(3) - US*ESL(3)
C
C---------- total source and doublet strength of segment
            SRC_U(L,IU)   = ADEL*US
            DBL_U(1,L,IU) = AAVG*UN(1)*DRLMAG*2.0
            DBL_U(2,L,IU) = AAVG*UN(2)*DRLMAG*2.0
            DBL_U(3,L,IU) = AAVG*UN(3)*DRLMAG*2.0
          ENDDO
 105    CONTINUE
 10   CONTINUE
C
      RETURN
      END ! SRDSET



 
      SUBROUTINE CROSS (U,V,W)
      REAL U(3), V(3), W(3)
      W(1) = U(2)*V(3) - U(3)*V(2)
      W(2) = U(3)*V(1) - U(1)*V(3)
      W(3) = U(1)*V(2) - U(2)*V(1)
      RETURN
      END      


      FUNCTION DOT (U,V)
      REAL U(3),V(3)
      DOT = U(1)*V(1) + U(2)*V(2) + U(3)*V(3)
      RETURN
      END





      SUBROUTINE VORVEL(X,Y,Z,LBOUND,X1,Y1,Z1,X2,Y2,Z2,BETA,
     &                   U,V,W )
C----------------------------------------------------------
C     Same as VORVEL1, with somewhat different formulation
C----------------------------------------------------------
      LOGICAL LBOUND
C
      REAL A(3), B(3), AXB(3)
C
      DATA PI4INV  / 0.079577472 /
C
      A(1) = (X1 - X)/BETA
      A(2) =  Y1 - Y
      A(3) =  Z1 - Z
C
      B(1) = (X2 - X)/BETA
      B(2) =  Y2 - Y
      B(3) =  Z2 - Z
C
      ASQ = A(1)**2 + A(2)**2 + A(3)**2
      BSQ = B(1)**2 + B(2)**2 + B(3)**2
C
      AMAG = SQRT(ASQ)
      BMAG = SQRT(BSQ)
C
      U = 0.
      V = 0.
      W = 0.
C
C---- contribution from the transverse bound leg
      IF (LBOUND .AND.  AMAG*BMAG .NE. 0.0) THEN
        AXB(1) = A(2)*B(3) - A(3)*B(2)
        AXB(2) = A(3)*B(1) - A(1)*B(3)
        AXB(3) = A(1)*B(2) - A(2)*B(1)
C
        ADB = A(1)*B(1) + A(2)*B(2) + A(3)*B(3)
C
        DEN = AMAG*BMAG + ADB
C
        T = (1.0/AMAG + 1.0/BMAG) / DEN
C
        U = AXB(1)*T
        V = AXB(2)*T
        W = AXB(3)*T
      ENDIF
C
C---- trailing leg attached to A
      IF (AMAG .NE. 0.0) THEN
        AXISQ = A(3)**2 + A(2)**2
C
        ADI = A(1)
        RSQ = AXISQ
C
        T = - (1.0 - ADI/AMAG) / RSQ
C
        V = V + A(3)*T
        W = W - A(2)*T
      ENDIF
C
C---- trailing leg attached to B
      IF (BMAG .NE. 0.0) THEN
        BXISQ = B(3)**2 + B(2)**2
C
        BDI = B(1)
        RSQ = BXISQ
C
        T =   (1.0 - BDI/BMAG) / RSQ
C
        V = V + B(3)*T
        W = W - B(2)*T
      ENDIF
C
      U = U*PI4INV / BETA
      V = V*PI4INV 
      W = W*PI4INV 
C
      RETURN
      END ! VORVEL




      SUBROUTINE VORVELC(X,Y,Z,LBOUND,X1,Y1,Z1,X2,Y2,Z2,BETA,
     &                   U,V,W, RCORE)
C----------------------------------------------------------
C     Same as VORVEL, with finite core radius
C----------------------------------------------------------
      LOGICAL LBOUND
C
C
      REAL A(3), B(3), AXB(3)
C
      DATA PI4INV  / 0.079577472 /
C
      A(1) = (X1 - X)/BETA
      A(2) =  Y1 - Y
      A(3) =  Z1 - Z
C
      B(1) = (X2 - X)/BETA
      B(2) =  Y2 - Y
      B(3) =  Z2 - Z
C
      ASQ = A(1)**2 + A(2)**2 + A(3)**2
      BSQ = B(1)**2 + B(2)**2 + B(3)**2
C
      AMAG = SQRT(ASQ)
      BMAG = SQRT(BSQ)
C
      U = 0.
      V = 0.
      W = 0.
C
C---- contribution from the transverse bound leg
      IF (LBOUND  .AND.  AMAG*BMAG .NE. 0.0) THEN
        AXB(1) = A(2)*B(3) - A(3)*B(2)
        AXB(2) = A(3)*B(1) - A(1)*B(3)
        AXB(3) = A(1)*B(2) - A(2)*B(1)
        AXBSQ = AXB(1)**2 + AXB(2)**2 + AXB(3)**2
C
        ADB = A(1)*B(1) + A(2)*B(2) + A(3)*B(3)
        ALSQ = ASQ + BSQ - 2.0*ADB
C
ccc     RSQ = AXBSQ / ALSQ
C
        AB = AMAG*BMAG
c        T = (AMAG+BMAG)*(1.0 - ADB/AB) / (AXBSQ + ALSQ*RCORE**2)
        T = (  (BSQ-ADB)/SQRT(BSQ+RCORE**2)
     &       + (ASQ-ADB)/SQRT(ASQ+RCORE**2) ) / (AXBSQ + ALSQ*RCORE**2)
C
        U = AXB(1)*T
        V = AXB(2)*T
        W = AXB(3)*T
      ENDIF
C
C---- trailing leg attached to A
      IF (AMAG .NE. 0.0) THEN
        AXISQ = A(3)**2 + A(2)**2
C
        ADI = A(1)
        RSQ = AXISQ
C
        T = - (1.0 - ADI/AMAG) / (RSQ + RCORE**2)
C
        V = V + A(3)*T
        W = W - A(2)*T
      ENDIF
C
C---- trailing leg attached to B
      IF (BMAG .NE. 0.0) THEN
        BXISQ = B(3)**2 + B(2)**2
C
        BDI = B(1)
        RSQ = BXISQ
C
        T =   (1.0 - BDI/BMAG) / (RSQ + RCORE**2)
C
        V = V + B(3)*T
        W = W - B(2)*T
      ENDIF
C
      U = U*PI4INV / BETA
      V = V*PI4INV 
      W = W*PI4INV 
C
      RETURN
      END ! VORVELC




      SUBROUTINE SRDVELC(X,Y,Z, X1,Y1,Z1, X2,Y2,Z2,
     &                   BETA,RCORE,
     &                   UVWS,UVWD  )
C-------------------------------------------------------------------
C     Same as SRDVEL, but with finite core radius
C-------------------------------------------------------------------
      REAL UVWS(3), UVWD(3,3)
C
      REAL R1(3), R2(3)
      REAL RXR(3)
C
      DATA PI4INV  / 0.079577472 /
C
      R1(1) = (X1-X)/BETA
      R1(2) =  Y1-Y
      R1(3) =  Z1-Z
C
      R2(1) = (X2-X)/BETA
      R2(2) =  Y2-Y
      R2(3) =  Z2-Z
C
      RCSQ = RCORE**2
C
      R1SQ = R1(1)**2 + R1(2)**2 + R1(3)**2
      R2SQ = R2(1)**2 + R2(2)**2 + R2(3)**2
C
      R1SQEPS = R1SQ + RCSQ
      R2SQEPS = R2SQ + RCSQ
C
      R1EPS = SQRT(R1SQEPS)
      R2EPS = SQRT(R2SQEPS)
C
      RDR = R1(1)*R2(1) + R1(2)*R2(2) + R1(3)*R2(3)
      RXR(1) = R1(2)*R2(3) - R1(3)*R2(2)
      RXR(2) = R1(3)*R2(1) - R1(1)*R2(3)
      RXR(3) = R1(1)*R2(2) - R1(2)*R2(1)
C
      XDX = RXR(1)**2 + RXR(2)**2 + RXR(3)**2
C
      ALL = R1SQ + R2SQ - 2.0*RDR
C
      DEN = RCSQ*ALL + XDX
C
      AI1 = ((RDR+RCSQ)/R1EPS - R2EPS)/DEN
      AI2 = ((RDR+RCSQ)/R2EPS - R1EPS)/DEN
C
C---- set velocity components for unit source and doublet
      DO K = 1, 3
        UVWS(K) = R1(K)*AI1 + R2(K)*AI2
C
        RR1 =  (R1(K)+R2(K))    /R1EPS 
     &        - R1(K)*(RDR+RCSQ)/R1EPS**3
     &        - R2(K)           /R2EPS
C
        RR2 =  (R1(K)+R2(K))    /R2EPS 
     &        - R2(K)*(RDR+RCSQ)/R2EPS**3
     &        - R1(K)           /R1EPS
C
        RRT = 2.0*R1(K)*(R2SQ  - RDR)
     &      + 2.0*R2(K)*(R1SQ  - RDR)
C
        AJ1 = (RR1 - AI1*RRT)/DEN
C
        AJ2 = (RR2 - AI2*RRT)/DEN
C
        DO J = 1, 3
          UVWD(K,J) = - AJ1*R1(J)
     &                - AJ2*R2(J)
        ENDDO
C
        UVWD(K,K) = UVWD(K,K) - AI1 - AI2
      ENDDO
C
      UVWS(1) = UVWS(1)*PI4INV/BETA
      UVWS(2) = UVWS(2)*PI4INV
      UVWS(3) = UVWS(3)*PI4INV
      DO L = 1, 3
        UVWD(1,L) = UVWD(1,L)*PI4INV/BETA
        UVWD(2,L) = UVWD(2,L)*PI4INV
        UVWD(3,L) = UVWD(3,L)*PI4INV
      ENDDO
C
      RETURN
      END ! SRDVELC





