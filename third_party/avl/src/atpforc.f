C***********************************************************************
C    Module:  atpforc.f
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

      SUBROUTINE TPFORC
C
C...PURPOSE  To calculate the far-field forces on the configuration using
C            a trefftz plane method.
C
C...INPUT    Geometry data from labelled commons
C            CNC     Strip span loading 
C
C...OUTPUT   CLFF    Total far-field lift
C            CYFF    Total far-field side force
C            CDFF    Total far-field drag
C            SPANEF  Span efficiency
C            DWWAKE  Far-field wake downwash at center of strip 
C
C...COMMENTS   The far-field drag is calculated using the Trefftz
C            plane (kinetic energy integral in the far wake).
C            The span-loading CNC is all that is required, plus
C            geometry data defining the wake position.
C            Since the wakes are just the horseshoe legs extending 
C            downstream from the bound legs, only the Y and Z 
C            coordinates are used. The normalwash in the cross-plane
C            is evaluated over the real and "image" sides.
C
      INCLUDE 'AVL.INC'
C
      REAL NY, NZ
      REAL VY_U(NUMAX), VZ_U(NUMAX), 
     &     VY_D(NDMAX), VZ_D(NDMAX), 
     &     VY_G(NGMAX), VZ_G(NGMAX)
      REAL P(3,3), P_M(3,3), P_A(3,3), P_B(3,3)
      REAL RT1(3,NSMAX),
     &     RT2(3,NSMAX),
     &     RTC(3,NSMAX)
      REAL GAMS(NSMAX), 
     &     GAMS_U(NSMAX,NUMAX),
     &     GAMS_D(NSMAX,NDMAX),
     &     GAMS_G(NSMAX,NGMAX)
C
      HPI = 1.0 / (2.0*PI)
C
C---- set Prandtl-Glauert transformation matrix
      ALFAT = 0.
      BETAT = 0.
      CALL PGMAT(AMACH,ALFAT,BETAT,P,P_M,P_A,P_B)
C
      YOFF = 2.0*YSYM
      ZOFF = 2.0*ZSYM
C
      CLFF = 0.
      CYFF = 0.
      CDFF = 0.
      DO N = 1, NUMAX
        CLFF_U(N) = 0.
        CYFF_U(N) = 0.
        CDFF_U(N) = 0.
      ENDDO
      DO N = 1, NCONTROL
        CLFF_D(N) = 0.
        CYFF_D(N) = 0.
        CDFF_D(N) = 0.
      ENDDO
      DO N = 1, NDESIGN
        CLFF_G(N) = 0.
        CYFF_G(N) = 0.
        CDFF_G(N) = 0.
      ENDDO
C
      DO JC = 1, NSTRIP
        GAMS(JC) = 0.
        DO N = 1, NUMAX
          GAMS_U(JC,N) = 0.
        ENDDO
        DO N = 1, NCONTROL
          GAMS_D(JC,N) = 0.
        ENDDO
        DO N = 1, NDESIGN
          GAMS_G(JC,N) = 0.
        ENDDO
C
        ISURF = NSURFS(JC)
        IF(LFLOAD(ISURF)) THEN
C------- add circulation of this strip only if it contributes to total load
         I1  = IJFRST(JC)
         NVC = NVSTRP(JC)
         DO I = I1, I1+NVC-1 
           GAMS(JC) = GAMS(JC) + GAM(I)
           DO N = 1, NUMAX
             GAMS_U(JC,N) = GAMS_U(JC,N) + GAM_U(I,N)
           ENDDO
           DO N = 1, NCONTROL
             GAMS_D(JC,N) = GAMS_D(JC,N) + GAM_D(I,N)
           ENDDO
           DO N = 1, NDESIGN
             GAMS_G(JC,N) = GAMS_G(JC,N) + GAM_G(I,N)
           ENDDO
         ENDDO
        ENDIF
      ENDDO
C
C---- set x,y,z in wind axes (Y,Z are then in Trefftz plane)
      DO JC = 1, NSTRIP
        IC = IJFRST(JC) + NVSTRP(JC) - 1
        DO K = 1, 3
          RT1(K,JC) = P(K,1)*RV1(1,IC)+P(K,2)*RV1(2,IC)+P(K,3)*RV1(3,IC)
          RT2(K,JC) = P(K,1)*RV2(1,IC)+P(K,2)*RV2(2,IC)+P(K,3)*RV2(3,IC)
          RTC(K,JC) = P(K,1)*RC (1,IC)+P(K,2)*RC (2,IC)+P(K,3)*RC (3,IC)
        ENDDO
      ENDDO
C
C...Find the normal velocity across each strip at the projected control
C   point location
      DO 40 JC = 1, NSTRIP
        DXT = RT2(1,JC) - RT1(1,JC)
        DYT = RT2(2,JC) - RT1(2,JC)
        DZT = RT2(3,JC) - RT1(3,JC)
        DST = SQRT(DYT*DYT + DZT*DZT)
C
        NY = -DZT / DST
        NZ =  DYT / DST
        YCNTR = RTC(2,JC)
        ZCNTR = RTC(3,JC)
C
        VY = 0.
        VZ = 0.
        DO N = 1, NUMAX
          VY_U(N) = 0.
          VZ_U(N) = 0.
        ENDDO
        DO N = 1, NCONTROL
          VY_D(N) = 0.
          VZ_D(N) = 0.
        ENDDO
        DO N = 1, NDESIGN
          VY_G(N) = 0.
          VZ_G(N) = 0.
        ENDDO
C
C...Sum velocity contributions from wake vortices
        DO 30 JV = 1, NSTRIP
          DSYZ = SQRT(  (RT2(2,JV)-RT1(2,JV))**2
     &                + (RT2(3,JV)-RT1(3,JV))**2 )
          IF(LSCOMP(NSURFS(JC)) .EQ. LSCOMP(NSURFS(JV))) THEN
ccc        RCORE = 0.0001*DSYZ
           RCORE = 0.
          ELSE
           RCORE = MAX( VRCORE*CHORD(JV) , 2.0*VRCORE*DSYZ )
          ENDIF
C
          RCORE = 0.
C
          DY1 = YCNTR - RT1(2,JV)
          DY2 = YCNTR - RT2(2,JV)
          DZ1 = ZCNTR - RT1(3,JV)
          DZ2 = ZCNTR - RT2(3,JV)
          RSQ1 = DY1*DY1 + DZ1*DZ1 + RCORE**2
          RSQ2 = DY2*DY2 + DZ2*DZ2 + RCORE**2
          VY = VY + HPI*GAMS(JV)*( DZ1/RSQ1 - DZ2/RSQ2)
          VZ = VZ + HPI*GAMS(JV)*(-DY1/RSQ1 + DY2/RSQ2)
          DO N = 1, NUMAX
            VY_U(N) = VY_U(N) + HPI*GAMS_U(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)
            VZ_U(N) = VZ_U(N) + HPI*GAMS_U(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)
          ENDDO
          DO N = 1, NCONTROL
            VY_D(N) = VY_D(N) + HPI*GAMS_D(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)
            VZ_D(N) = VZ_D(N) + HPI*GAMS_D(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)
          ENDDO
          DO N = 1, NDESIGN
            VY_G(N) = VY_G(N) + HPI*GAMS_G(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)
            VZ_G(N) = VZ_G(N) + HPI*GAMS_G(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)
          ENDDO
C
          IF(IZSYM.NE.0) THEN
            DY1 = YCNTR -       RT1(2,JV)
            DY2 = YCNTR -       RT2(2,JV)
            DZ1 = ZCNTR - (ZOFF-RT1(3,JV))
            DZ2 = ZCNTR - (ZOFF-RT2(3,JV))
CCC         DZ1 = ZCNTR - (ZOFF-RT1(3,JV)+ALFA*RT1(1,JV))
CCC         DZ2 = ZCNTR - (ZOFF-RT2(3,JV)+ALFA*RT2(1,JV))
            RSQ1 = DY1*DY1 + DZ1*DZ1
            RSQ2 = DY2*DY2 + DZ2*DZ2
            VY = VY - HPI*GAMS(JV)*( DZ1/RSQ1 - DZ2/RSQ2)*IZSYM
            VZ = VZ - HPI*GAMS(JV)*(-DY1/RSQ1 + DY2/RSQ2)*IZSYM
            DO N = 1, NUMAX
              VY_U(N) = VY_U(N)
     &          - HPI*GAMS_U(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IZSYM
              VZ_U(N) = VZ_U(N)
     &          - HPI*GAMS_U(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IZSYM
            ENDDO
            DO N = 1, NCONTROL
              VY_D(N) = VY_D(N)
     &          - HPI*GAMS_D(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IZSYM
              VZ_D(N) = VZ_D(N)
     &          - HPI*GAMS_D(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IZSYM
            ENDDO
            DO N = 1, NDESIGN
              VY_G(N) = VY_G(N)
     &          - HPI*GAMS_G(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IZSYM
              VZ_G(N) = VZ_G(N)
     &          - HPI*GAMS_G(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IZSYM
            ENDDO
          ENDIF 
C
          IF(IYSYM.NE.0) THEN
            DY1 = YCNTR - (YOFF-RT1(2,JV))
            DY2 = YCNTR - (YOFF-RT2(2,JV))
            DZ1 = ZCNTR -       RT1(3,JV)
            DZ2 = ZCNTR -       RT2(3,JV)
            RSQ1 = DY1*DY1 + DZ1*DZ1
            RSQ2 = DY2*DY2 + DZ2*DZ2
            VY = VY - HPI*GAMS(JV)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM
            VZ = VZ - HPI*GAMS(JV)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM
            DO N = 1, NUMAX
              VY_U(N) = VY_U(N)
     &          - HPI*GAMS_U(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM
              VZ_U(N) = VZ_U(N)
     &          - HPI*GAMS_U(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM
            ENDDO
            DO N = 1, NCONTROL
              VY_D(N) = VY_D(N)
     &          - HPI*GAMS_D(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM
              VZ_D(N) = VZ_D(N)
     &          - HPI*GAMS_D(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM
            ENDDO
            DO N = 1, NDESIGN
              VY_G(N) = VY_G(N)
     &          - HPI*GAMS_G(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM
              VZ_G(N) = VZ_G(N)
     &          - HPI*GAMS_G(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM
            ENDDO
C
            IF(IZSYM.NE.0) THEN
              DY1 = YCNTR - (YOFF-RT1(2,JV))
              DY2 = YCNTR - (YOFF-RT2(2,JV))
              DZ1 = ZCNTR - (ZOFF-RT1(3,JV))
              DZ2 = ZCNTR - (ZOFF-RT2(3,JV))
CCC           DZ1 = ZCNTR - (ZOFF-RT1(3,JV)+ALFA*RT1(1,JV))
CCC           DZ2 = ZCNTR - (ZOFF-RT2(3,JV)+ALFA*RT2(1,JV))
              RSQ1 = DY1*DY1 + DZ1*DZ1
              RSQ2 = DY2*DY2 + DZ2*DZ2
              VY = VY + HPI*GAMS(JV)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM*IZSYM
              VZ = VZ + HPI*GAMS(JV)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM*IZSYM
              DO N = 1, NUMAX
                VY_U(N) = VY_U(N)
     &            - HPI*GAMS_U(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM*IZSYM
                VZ_U(N) = VZ_U(N)
     &            - HPI*GAMS_U(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM*IZSYM
              ENDDO
              DO N = 1, NCONTROL
                VY_D(N) = VY_D(N)
     &            - HPI*GAMS_D(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM*IZSYM
                VZ_D(N) = VZ_D(N)
     &            - HPI*GAMS_D(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM*IZSYM
              ENDDO
              DO N = 1, NDESIGN
                VY_G(N) = VY_G(N)
     &            - HPI*GAMS_G(JV,N)*( DZ1/RSQ1 - DZ2/RSQ2)*IYSYM*IZSYM
                VZ_G(N) = VZ_G(N)
     &            - HPI*GAMS_G(JV,N)*(-DY1/RSQ1 + DY2/RSQ2)*IYSYM*IZSYM
              ENDDO
            ENDIF
          ENDIF
C
   30   CONTINUE
C
C...Trefftz-plane drag is kinetic energy in crossflow
        DWWAKE(JC) = -(NY*VY + NZ*VZ)
C
        CLFF = CLFF + 2.0*GAMS(JC)*          DYT    /SREF
        CYFF = CYFF - 2.0*GAMS(JC)* DZT             /SREF
        CDFF = CDFF +     GAMS(JC)*(DZT*VY - DYT*VZ)/SREF
        DO N = 1, NUMAX
          CLFF_U(N) = CLFF_U(N) + 2.0*GAMS_U(JC,N)*DYT/SREF
          CYFF_U(N) = CYFF_U(N) - 2.0*GAMS_U(JC,N)*DZT/SREF
          CDFF_U(N) = CDFF_U(N)
     &              + (  GAMS_U(JC,N)*(DZT*VY      - DYT*VZ     )
     &                 + GAMS(JC)    *(DZT*VY_U(N) - DYT*VZ_U(N)) )/SREF
        ENDDO
        DO N = 1, NCONTROL
          CLFF_D(N) = CLFF_D(N) + 2.0*GAMS_D(JC,N)*DYT/SREF
          CYFF_D(N) = CYFF_D(N) - 2.0*GAMS_D(JC,N)*DZT/SREF
          CDFF_D(N) = CDFF_D(N)
     &              + (  GAMS_D(JC,N)*(DZT*VY      - DYT*VZ     )
     &                 + GAMS(JC)    *(DZT*VY_D(N) - DYT*VZ_D(N)) )/SREF
        ENDDO
        DO N = 1, NDESIGN
          CLFF_G(N) = CLFF_G(N) + 2.0*GAMS_G(JC,N)*DYT/SREF
          CYFF_G(N) = CYFF_G(N) - 2.0*GAMS_G(JC,N)*DZT/SREF
          CDFF_G(N) = CDFF_G(N)
     &              + (  GAMS_G(JC,N)*(DZT*VY      - DYT*VZ     )
     &                 + GAMS(JC)    *(DZT*VY_G(N) - DYT*VZ_G(N)) )/SREF
        ENDDO
   40 CONTINUE
C
C---- Double the X,Z forces, zero Y force for a Y symmetric case
      IF(IYSYM.EQ.1) THEN
       CLFF = 2.0 * CLFF
       CYFF = 0.0
       CDFF = 2.0 * CDFF
       DO N = 1, NUMAX
         CLFF_U(N) = 2.0 * CLFF_U(N)
         CYFF_U(N) = 0.0
         CDFF_U(N) = 2.0 * CDFF_U(N)
       ENDDO
       DO N = 1, NCONTROL
         CLFF_D(N) = 2.0 * CLFF_D(N)
         CYFF_D(N) = 0.0
         CDFF_D(N) = 2.0 * CDFF_D(N)
       ENDDO
       DO N = 1, NDESIGN
         CLFF_G(N) = 2.0 * CLFF_G(N)
         CYFF_G(N) = 0.0
         CDFF_G(N) = 2.0 * CDFF_G(N)
       ENDDO
      ENDIF
C
C---- aspect ratio
      AR = BREF**2 / SREF
C
C---- span efficiency
      IF(CDFF .EQ. 0.0) THEN
       SPANEF = 0.
       SPANEF_A = 0.
       DO N = 1, NUMAX
         SPANEF_U(N) = 0.
       ENDDO
       DO N = 1, NCONTROL
         SPANEF_D(N) = 0.
       ENDDO
       DO N = 1, NDESIGN
         SPANEF_G(N) = 0.
       ENDDO
C
      ELSE
       SPANEF = (CLFF**2 + CYFF**2)/ (PI * AR * CDFF)
       SPANEF_CL = 2.0*CLFF / (PI * AR * CDFF)
       SPANEF_CY = 2.0*CYFF / (PI * AR * CDFF)
       SPANEF_CD = -SPANEF/CDFF
C
       SPANEF_A = 0.
       DO N = 1, NUMAX
         SPANEF_U(N) = SPANEF_CL*CLFF_U(N)
     &               + SPANEF_CY*CYFF_U(N)
     &               + SPANEF_CD*CDFF_U(N)
       ENDDO
       DO N = 1, NCONTROL
         SPANEF_D(N) = SPANEF_CL*CLFF_D(N)
     &               + SPANEF_CY*CYFF_D(N)
     &               + SPANEF_CD*CDFF_D(N)
       ENDDO
       DO N = 1, NDESIGN
         SPANEF_G(N) = SPANEF_CL*CLFF_G(N)
     &               + SPANEF_CY*CYFF_G(N)
     &               + SPANEF_CD*CDFF_G(N)
       ENDDO
      ENDIF
C
      RETURN
      END ! TPFORC



      SUBROUTINE PGMAT(MACH,ALFA,BETA,P,P_M,P_A,P_B)
C-------------------------------------------------------
C     Calculates Prandtl-Glauert transformation matrix.
C      
C      xi      [       ] x
C              [       ]  
C      eta  =  [   P   ] y
C              [       ]
C      zeta    [       ] z
C
C-------------------------------------------------------
C
      REAL MACH, ALFA, BETA
      REAL P(3,3), P_M(3,3), P_A(3,3), P_B(3,3)
C
      BINV = 1.0 / SQRT(1.0 - MACH**2)
      BI_M = MACH * BINV**3
C
      SINA = SIN(ALFA)
      COSA = COS(ALFA)
C
      SINB = SIN(BETA)
      COSB = COS(BETA)
C
C
      P(1,1) =  COSA*COSB*BINV
      P(1,2) =      -SINB*BINV
      P(1,3) =  SINA*COSB*BINV
C
      P(2,1) =  COSA*SINB
      P(2,2) =       COSB
      P(2,3) =  SINA*SINB
C
      P(3,1) = -SINA
      P(3,2) = 0.
      P(3,3) =  COSA
C
C
      P_M(1,1) =  COSA*COSB*BI_M
      P_M(1,2) =      -SINB*BI_M
      P_M(1,3) =  SINA*COSB*BI_M
C
      P_M(2,1) = 0.
      P_M(2,2) = 0.
      P_M(2,3) = 0.
C
      P_M(3,1) = 0.
      P_M(3,2) = 0.
      P_M(3,3) = 0.
C
C
      P_A(1,1) = -SINA*COSB*BINV
      P_A(1,2) = 0.
      P_A(1,3) =  COSA*COSB*BINV
C
      P_A(2,1) = -SINA*SINB
      P_A(2,2) = 0.
      P_A(2,3) =  COSA*SINB
C
      P_A(3,1) = -COSA
      P_A(3,2) = 0.
      P_A(3,3) = -SINA
C
C
      P_B(1,1) = -COSA*SINB*BINV
      P_B(1,2) =      -COSB*BINV
      P_B(1,3) = -SINA*SINB*BINV
C
      P_B(2,1) =  COSA*COSB
      P_B(2,2) =      -SINB
      P_B(2,3) =  SINA*COSB
C
      P_B(3,1) = 0.
      P_B(3,2) = 0.
      P_B(3,3) = 0.
C
      RETURN
      END ! PGMAT
