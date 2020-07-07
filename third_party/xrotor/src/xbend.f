C***********************************************************************
C    Module:  xbend.f
C 
C    Copyright (C) 2011 Mark Drela 
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
C
      SUBROUTINE BEND
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 COMAND
      CHARACTER*132 COMARG
C
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR
C
C---------------------------------------------
C     Run rotor at arbitrary operating points
C---------------------------------------------
C
      GREEK = .FALSE.
C
      IF(LSTRUC) THEN
       WRITE(*,*)
       WRITE(*,*) 'Structural properties available'
      ELSE
       WRITE(*,*)
       WRITE(*,*) 'Structural properties not available'
      ENDIF
C
 900  CALL ASKC('.BEND^',COMAND,COMARG)
C
      DO I=1, 20
        IINPUT(I) = 0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
      IF(COMAND.EQ.'    ') RETURN
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'READ') GO TO 10
      IF(COMAND.EQ.'CLR ') GO TO 20
      IF(COMAND.EQ.'EVAL') GO TO 30
      IF(COMAND.EQ.'DEFL') GO TO 40
      IF(COMAND.EQ.'REST') GO TO 50
      IF(COMAND.EQ.'PLOT') GO TO 60
      IF(COMAND.EQ.'SIZE') GO TO 62
      IF(COMAND.EQ.'HARD') GO TO 65
      IF(COMAND.EQ.'WRIT') GO TO 70
      IF(COMAND.EQ.'SETS') GO TO 80
      IF(COMAND.EQ.'MCLR') GO TO 85
      IF(COMAND.EQ.'ANNO') GO TO 90
      IF(COMAND.EQ.'HELP') GO TO 100
C
C-------------------------------------------------------------
      WRITE(*,1000) COMAND
      GO TO 900
C
C-------------------------------------------------------------
 10   CALL EILOAD(COMARG)
      GO TO 900
C
C-------------------------------------------------------------
 20   CALL STCLR
      GO TO 900
C
C-------------------------------------------------------------
 30   IF(.NOT.LSTRUC) THEN
       WRITE(*,*) 'Structural properties not available'
       WRITE(*,*) 'Assuming zero mass, infinite stiffness...'
      ENDIF
ccc      CALL STLOAD
      CALL STCALC
      CALL STWRIT(LUWRIT)
      GO TO 900
C
C-------------------------------------------------------------
 40   CALL STADD
      GO TO 900
C
C-------------------------------------------------------------
 50   DO 51 I=1, II
        BETA(I) = BETA0(I)
 51   CONTINUE
      CONV = .FALSE.
      GO TO 900
C
C-------------------------------------------------------------
 60   CONTINUE
      IF(NINPUT.GE.1) THEN
       NPLOT = IINPUT(1)
      ELSE
       WRITE(*,2000) 
       CALL ASKI('Select plot number^',NPLOT)
      ENDIF
C
      IF(NPLOT.EQ.0) THEN
        GO TO 900
      ELSE IF(NPLOT.EQ.1) THEN
        CALL EIPLOT
      ELSE IF(NPLOT.EQ.2) THEN
        CALL STPLOT
ccc   ELSE IF(NPLOT.EQ.3) THEN
ccc     CALL DEFPLT
      ELSE
        NINPUT = 0
        GO TO 60
      ENDIF
      GO TO 900
C
C-------------------------------------------------------------
 62   IF(NINPUT.GE.1) THEN
       SIZE = RINPUT(1)
      ELSE
       WRITE(*,*) 'Current plot size = ', SIZE
       CALL ASKR('Enter new plot size^',SIZE)
      ENDIF
      GO TO 900
C
C-------------------------------------------------------------
 65   IF(LPLOT) THEN
       CALL PLEND
       CALL REPLOT(IDEVRP)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C-------------------------------------------------------------
 70   IF(COMARG(1:1).NE.' ') SAVFIL = COMARG
      CALL OPFILE(LUSAVE,SAVFIL)
      CALL STWRIT(LUSAVE)
      CLOSE(LUSAVE)
      GO TO 900
C
C-------------------------------------------------------------
 80   CALL STSET
      GO TO 900
C
C-------------------------------------------------------------
 85   CALL MCLR
      GO TO 900
C
C-------------------------------------------------------------
 90   IF(LPLOT) THEN
       CALL ANNOT(1.2*CSIZE)
      ELSE
       WRITE(*,*) 'No current plot'
      ENDIF
      GO TO 900
C
C-------------------------------------------------------------
 100  WRITE(*,3000)
      GO TO 900
C
C.......................................................................
C
 1000 FORMAT(1X,A4,' command not recognized.  Type a "?" for list')
 1100 FORMAT(
     &  /'   READ f Read in blade structural properties'
     &  /'   EVAL   Evaluate structural loads and deflections'
     &  /'   CLR    Clear all structural deflections'
     & //'   DEFL   Set new twist  =  static  +  structural twist'
     &  /'   REST   Set new twist  =  static twist'
     &  /'   SETS   Set static twist = current - structural twist' 
     & //'   WRIT f Write structural solution to disk file'
     & //'   PLOT i Plot structural parameters'
     &  /'   ANNO   Annotate plot'
     &  /'   HARD   Hardcopy current plot'
     &  /'   SIZE r Change plot-object size'
     & //'   HELP   Display help on structural calculation')
C
 2000 FORMAT(/'  0   CANCEL'
     &       /'  1   Structural properties'
     &       /'  2   Loads and material strains')
ccc  &       /'  3   Deflections'
C
 3000 FORMAT(/
     & 'The axis definitions are:'//
     & '  X  aft along prop rotation axis'/
     & '  Y  radial alng blade'/
     & '  Z  perpendicular to blade:    X x Y = Z'//
     & 'The structural solution contains two groups of data, '
     & 'the first group has:'//
     & '  u/R    deflections in the X direction'/
     & '  w/R    deflections in the Z direction'/
     & '   t     torsional twist (positive in the increasing '
     & 'incidence direction)'/
     & '   Mz    bending moment about the Z axis'/
     & '   Mx    bending moment about the X axis'/
     & '   T     moment about the radial Y axis (i.e. torsion)'/
     & '   P     tensile load (shear in Y direction)'/
     & '   Sx    shear in X direction'/
     & '   Sz    shear in Z direction'//
     & 'The second group of structural data contains:'//
     & '   Ex    strain due to bending in the X direction'/
     & '   Ey    strain due to extension in the Y direction'/
     & '   Ez    strain due to bending in the Z direction'/
     & '   Emax  maximum strain calculated by  '
     & 'Emax = sqrt(Ex^2 + Ez^2) + Ey'/
     & '   g     shear strain due to twist t'//
     & '   Note that Ex, Ez, and g are evaluated at the '
     & 'local radius RST from'/
     & '   the structural axis (RST is an input quantity, '
     & 'normally set to the '/
     & '   distance of the highest or lowest profile point '
     & 'from the structural axis)'/)
C
      END ! BEND



      SUBROUTINE EILOAD(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
      DIMENSION XT(IX)
C----------------------------------------------------
C     Reads and splines blade structural properties.
C----------------------------------------------------
C
C    XT =    R    radius
C    W0 = EIXX    in-plane stiffness
C    W1 = EIYY    out-of-plane stiffness
C    W2 =   EA    extensional stiffness
C    W3 =   GJ    torsional stiffness
C    W4 =   EK    extensional/torsional cross-stiffness
C    W5 =    M    mass density / length
C    W6 =  MXX    pitch axis inertia / length
C    W7 = XOCG    x/c of section CG
C    W8 = XOSC    x/c of section shear center (structural axis)
C    W9 = RST     structural radius for strain evaluation
C
      LU = 14
C
      FNAME = FNAME1
      IF(FNAME(1:1).EQ.' ') CALL ASKS('Enter input filename^',FNAME)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=200)
      READ(LU,1000) DUMMY
      READ(LU,1000) DUMMY
      READ(LU,1000) DUMMY
      DO 10 IT=1, IX
        READ(LU,*,END=11,ERR=210) XT(IT),
     &     W0(IT), W1(IT), W2(IT), W3(IT), W4(IT),
     &     W5(IT), W6(IT), W7(IT), W8(IT), W9(IT)
        XT(IT) = XT(IT)/RAD
 10   CONTINUE
      WRITE(*,*) 'EILOAD: Array overflow.  Too many radial stations.'
 11   CONTINUE
      NT = IT-1
      CLOSE(LU)
C
      CALL SEGSPL(W0,T0,XT,NT)
      CALL SEGSPL(W1,T1,XT,NT)
      CALL SEGSPL(W2,T2,XT,NT)
      CALL SEGSPL(W3,T3,XT,NT)
      CALL SEGSPL(W4,T4,XT,NT)
      CALL SEGSPL(W5,T5,XT,NT)
      CALL SEGSPL(W6,T6,XT,NT)
      CALL SEGSPL(W7,T7,XT,NT)
      CALL SEGSPL(W8,T8,XT,NT)
      CALL SEGSPL(W9,T9,XT,NT)
C
      DO 30 I=1, II
        EIXXB(I) = SEVAL(XI(I),W0,T0,XT,NT)
        EIYYB(I) = SEVAL(XI(I),W1,T1,XT,NT)
        EAB(I)   = SEVAL(XI(I),W2,T2,XT,NT)
        GJB(I)   = SEVAL(XI(I),W3,T3,XT,NT)
        EKB(I)   = SEVAL(XI(I),W4,T4,XT,NT)
        MB(I)    = SEVAL(XI(I),W5,T5,XT,NT)
        MXXB(I)  = SEVAL(XI(I),W6,T6,XT,NT)
        XOCG(I)  = SEVAL(XI(I),W7,T7,XT,NT)
        XOSC(I)  = SEVAL(XI(I),W8,T8,XT,NT)
        RSTB(I)  = SEVAL(XI(I),W9,T9,XT,NT)
 30   CONTINUE
C
      MASS = 0.0
      MRSQ = 0.0
      MAXX = 0.0
      DO 50 I=1, II
        MASS = MASS +   MB(I)*RAD*DXI(I)
        MRSQ = MRSQ +   MB(I)*RAD*DXI(I) * (XI(I)*RAD)**2
        MAXX = MAXX + MXXB(I)*RAD*DXI(I)
 50   CONTINUE
C
      WRITE(*,3100) MASS, MAXX, MRSQ
C
      LSTRUC = .TRUE.
      RETURN
C
  200 WRITE(*,1010) FNAME(1:32)
      RETURN
C
  210 WRITE(*,1020) FNAME(1:32)
      CLOSE(LU)
      CONV = .FALSE.
      RETURN
C..............................
 1000 FORMAT(32A1)
 1010 FORMAT(' File  ',A,' not found'/)
 1020 FORMAT(' File  ',A,' has incompatible format'/
     &       ' Loading not completed'/)
 3100 FORMAT(/' Blade mass =', G12.4
     &       /' Pitch-axis inertia =',G14.5
     &       /' Rotational inertia =',G14.5)
      END ! EILOAD


      SUBROUTINE STCLR
      INCLUDE 'XROTOR.INC'
C
      DO 10 I=1, II
        TX(I) = 0.0
        TY(I) = 0.0
        TZ(I) = 0.0
        WX(I) = 0.0
        WY(I) = 0.0
        WZ(I) = 0.0
        MOMX(I) = 0.0
        MOMY(I) = 0.0
        MOMZ(I) = 0.0
        SHRX(I) = 0.0
        SHRY(I) = 0.0
        SHRZ(I) = 0.0
 10   CONTINUE
C
      RETURN
      END ! STCLR


      SUBROUTINE MCLR
      INCLUDE 'XROTOR.INC'
C
      DO 10 I=1, II
        MB(I) = 0.0
        MXXB(I) = 0.0
        EKB(I) = 0.0
 10   CONTINUE
C
      RETURN
      END ! MCLR



      SUBROUTINE STLOAD
      INCLUDE 'XROTOR.INC'
C-----------------------------------------------------------
C     Calculates force and moment loadings along blade.
C HHY 3/99 a local x',y',z' system is assumed that is tilted by 
C          a rake angle about the z axis, the y' axis runs out 
C          the blade axis.  z' = z, x',y' are rotated from x,y by rake angle
C
C     The ()_A and ()_B sensitivities below should be set 
C     only if BETA(.) and the aero solution CL(.), GAM(.), 
C     etc. are updated every iteration in STCALC.
C-----------------------------------------------------------
C
      FX = 0.0
      FY = 0.0
      FZ = 0.0
      SINR = SIN(RAKE)
      COSR = COS(RAKE)
C
      DO 10 I=1, II
C
        DXII = DXI(I)/COSR
C
        CALL CSCALC(I,UTOT,WA,WT,
     &              VT,VT_ADW,
     &              VA,VA_ADW,
     &              VD,VD_ADW,
     &              CI,CI_ADV,CI_VT,
     &              SI,             SI_VA,
     &              W,  W_ADV, W_VT, W_VA,
     &              PHI,P_ADV, P_VT, P_VA)
C
        WSQ = W*W
C
        SINB = SIN(BETA(I))
        COSB = COS(BETA(I))
C
        TXA = 0.5*(TX(I) + TX(I+1))
        TZA = 0.5*(TZ(I) + TZ(I+1))
        WZA = 0.5*(WZ(I) + WZ(I+1))
C
C--- Aerodynamic lift and drag forces (act in blade normal plane)
        FLIFT = 0.5*WSQ*CH(I)*CL(I)
        FDRAG = 0.5*WSQ*CH(I)*CD(I)
C
        FLIFT_A = 0.0
ccc     FLIFT_A = 0.5*WSQ*CH(I)*DCLDA
C--- blade resolved force components
        FXA = -CI/W*FLIFT + SI/W*FDRAG
        FYA =  0.0
        FZA =  SI/W*FLIFT + CI/W*FDRAG
C
C***************************************
C--- Integrate blade aerodynamic forces
       FX = FX + COSR*DXII*FXA
       FY = FY + SINR*DXII*FXA
       FZ = FZ + DXII*FZA
C***************************************
C
C
C--- Centrifugal forces (y direction)
        FCENT = 0.0
        IF(LSTRUC) FCENT = MB(I)*XI(I)/ADV**2
     &                   / (RHO * RAD**2)
C--- blade resolved force components
        FXC = -SINR*FCENT 
        FYC =  COSR*FCENT 
        FZC =  0.0
C
C--- Assemble force loadings
        PX(I) =  FXA + FXC + FYC*TZA*COSR
        PY(I) =  FYA + FYC 
        PZ(I) =  FZA + FZC + FYC*(WZA/XI(I) - TXA)
        PX_TY(I) = -FLIFT_A*CI/W
        PX_TZ(I) =  FYC*COSR
        PZ_TX(I) = -FYC
        PZ_TY(I) =  FLIFT_A*SI/W
        PZ_WZ(I) =  FYC/XI(I)
c
c        PX(I) =  -FLIFT*CI/W + FDRAG*SI/W + FCENT* TZA
c        PZ(I) =   FLIFT*SI/W + FDRAG*CI/W + FCENT*(WZA/XI(I) - TXA)
c        PY(I) =                             FCENT
c        PX_TY(I) = -FLIFT_A*CI/W
c        PX_TZ(I) =  FCENT
c        PZ_TX(I) = -FCENT
c        PZ_TY(I) =  FLIFT_A*SI/W
c        PZ_WZ(I) =  FCENT/XI(I)
C
C--- Define moment components
        IF(LSTRUC) THEN
         MCENT   = SINB * (XOCG(I)-XOSC(I))*CH(I) * MB(I)*XI(I)/ADV**2
     &           / (RHO * RAD**2)
         MPREC   = -SINB*COSB * MXXB(I)/ADV**2
     &           / (RHO * RAD**4)
         MAERO   = 0.5*WSQ*(CL(I)*(XOSC(I)-0.25) + CM(I)) * CH(I)**2
        ELSE
         MCENT = 0.0
         MPREC = 0.0
         MAERO   = 0.5*WSQ*(CL(I)*(0.40-0.25) + CM(I)) * CH(I)**2
        ENDIF
C
        MCENT_B = 0.0
        MPREC_B = 0.0
        MAERO_A = 0.0
ccc     MCENT_B = COSB * (XOCG(I)-XOSC(I))*CH(I) * MB(I)*XI(I)/ADV**2
ccc  &          / (RHO * RAD**2)
ccc     MPREC_B = -(COSB**2 - SINB**2) * MXXB(I)/ADV**2
ccc  &          / (RHO * RAD**4)
ccc     MAERO_A = 0.5*WSQ*(DCLDA*(XOSC(I)-0.25)     ) * CH(I)**2
C
C--- Assemble imposed moments
        MX(I) = 0.0
        MY(I) = MAERO + MPREC
        MZ(I) = MCENT + MPREC*(WZA/XI(I) - TXA)
C
        MY_TY(I) = MAERO_A + MPREC_B*(WZA/XI(I) - TXA)
        MZ_TY(I) = MCENT_B
        MZ_TX(I) =         - MPREC
        MZ_WZ(I) =           MPREC/XI(I)
C
 10   CONTINUE
C
C--- Print the blade aerodynamic forces
      WRITE(*,20) FX* RHO * VEL**2 * RAD**2,
     &            FY* RHO * VEL**2 * RAD**2,
     &            FZ* RHO * VEL**2 * RAD**2
 20   FORMAT(/'Blade aerodynamic forces:',
     &       /' FX (axial)      = ',F12.6,
     &       /' FY (radial)     = ',F12.6,
     &       /' FZ (tangential) = ',F12.6)
C
      RETURN
      END ! STLOAD



      SUBROUTINE STCALC
      INCLUDE 'XROTOR.INC'
C------------------------------------------------------------
C     Updates resultants and deflections along blade.
C     Uses current loading distributions PX,PY,PZ, MX,MY,MZ.
C------------------------------------------------------------
      REAL AA(12,12,IXP), BB(12,12,IXP), CC(12,12,IXP), RR(12,IXP)
      REAL RRLIM(12), RLXR(12)
C
      EPS = 1.0E-5
C
      RRLIM(1) = 0.10
      RRLIM(2) = 0.10
      RRLIM(3) = 0.10
C
      RRLIM(4) = 0.2/ADV**2
      RRLIM(5) = 0.5/ADV**2
      RRLIM(6) = 0.2/ADV**2
C
      RRLIM(7) = 0.2 /ADV**2
      RRLIM(8) = 20.0/ADV**2
      RRLIM(9) = 0.2 /ADV**2
C
      RRLIM(10) = 0.10
      RRLIM(11) = 0.01
      RRLIM(12) = 0.10
C
      WRITE(*,*)
C
C---- Newton iteration loop
      DO 100 ITER=1, 10
C
      CALL STLOAD
C
      DO 8 I=1, II+1
        DO 81 K=1, 12
          DO 811 J=1, 12
            AA(K,J,I) = 0.0
            BB(K,J,I) = 0.0
            CC(K,J,I) = 0.0
 811      CONTINUE
          RR(K,I) = 0.0
 81     CONTINUE
 8    CONTINUE
C
C
C---- fix deflection angles at root
      I = 1
      AA(1,1,I) = 1.0
      AA(2,2,I) = 1.0
      AA(3,3,I) = 1.0
C
      AA(10,10,I) = 1.0
      AA(11,11,I) = 1.0
      AA(12,12,I) = 1.0
C
C---- non-dimensionalizing factors
      EAREF = RHO * VEL**2 * RAD**2
      EKREF = RHO * VEL**2 * RAD**3
      EIREF = RHO * VEL**2 * RAD**4
C
      COSR = COS(RAKE)
C
C---- go over radial intervals
      DO 10 I=1, II
C
        DXII = DXI(I)/COSR
C
        SX2 = SHRX(I+1)
        SY2 = SHRY(I+1)
        SZ2 = SHRZ(I+1)
C
        SX1 = SHRX(I)
        SY1 = SHRY(I)
        SZ1 = SHRZ(I)
C
        MX2 = MOMX(I+1)
        MY2 = MOMY(I+1)
        MZ2 = MOMZ(I+1)
C
        MX1 = MOMX(I)
        MY1 = MOMY(I)
        MZ1 = MOMZ(I)
C
        TX2 = TX(I+1)
        TY2 = TY(I+1)
        TZ2 = TZ(I+1)
C
        TX1 = TX(I)
        TY1 = TY(I)
        TZ1 = TZ(I)
C
        WX2 = WX(I+1)
        WY2 = WY(I+1)
        WZ2 = WZ(I+1)
C
        WX1 = WX(I)
        WY1 = WY(I)
        WZ1 = WZ(I)
C
C
c        PX_TY(I) = -FLIFT_A*CI/W
c        PX_TZ(I) =  FCENT
c        PZ_TX(I) = -FCENT
c        PZ_TY(I) =  FLIFT_A*SI/W
c        PZ_WZ(I) =  FCENT/XI(I)
c
c        MY_TY(I) = MAERO_A + MPREC_B*(WZA/XI(I) - TXA)
c        MZ_TY(I) = MCENT_B
c        MZ_TX(I) =         - MPREC
c        MZ_WZ(I) =           MPREC/XI(I)
C

C------ x-moment
        RR(4,I)   =   MX2-MX1
     &             - (MY2+MY1)*0.5*(TZ2-TZ1)
     &             + (MZ2+MZ1)*0.5*(TY2-TY1)
     &             -((SZ2+SZ1)*0.5 - MX(I))  *DXII
C
        AA(4,2,I) = -(MZ2+MZ1)*0.5
        AA(4,3,I) =  (MY2+MY1)*0.5
        AA(4,4,I) =      -1.0
        AA(4,5,I) =           -0.5*(TZ2-TZ1)
        AA(4,6,I) =            0.5*(TY2-TY1)
        AA(4,9,I) =           -0.5           *DXII
        CC(4,2,I) =  (MZ2+MZ1)*0.5
        CC(4,3,I) = -(MY2+MY1)*0.5
        CC(4,4,I) =   1.0
        CC(4,5,I) =           -0.5*(TZ2-TZ1)
        CC(4,6,I) =            0.5*(TY2-TY1)
        CC(4,9,I) =           -0.5           *DXII
C
C------ y-moment
        RR(5,I)   =   MY2-MY1
     &             + (MX2+MX1)*0.5*(TZ2-TZ1)
     &             - (MZ2+MZ1)*0.5*(TX2-TX1)
     &          + (                  MY(I))  *DXII
        AA(5,1,I) =  (MZ2+MZ1)*0.5
        AA(5,2,I) =                  MY_TY(I)*DXII * 0.5
        AA(5,3,I) = -(MX2+MX1)*0.5
        AA(5,4,I) =            0.5*(TZ2-TZ1)
        AA(5,5,I) =      -1.0
        AA(5,6,I) =           -0.5*(TX2-TX1)
        CC(5,1,I) = -(MZ2+MZ1)*0.5
        CC(5,2,I) =                  MY_TY(I)*DXII * 0.5
        CC(5,3,I) =  (MX2+MX1)*0.5
        CC(5,4,I) =            0.5*(TZ2-TZ1)
        CC(5,5,I) =   1.0
        CC(5,6,I) =           -0.5*(TX2-TX1)
C
C------ z-moment
        RR(6,I)   =   MZ2-MZ1
     &             + (MY2+MY1)*0.5*(TX2-TX1)
     &             - (MX2+MX1)*0.5*(TY2-TY1)
     &             +((SX2+SX1)*0.5 + MZ(I))  *DXII
        AA(6,1,I) = -(MY2+MY1)*0.5
        AA(6,2,I) =  (MX2+MX1)*0.5 + MZ_TY(I)*DXII * 0.5
        AA(6,3,I) =                  MZ_TX(I)*DXII * 0.5
        AA(6,4,I) =           -0.5*(TY2-TY1)
        AA(6,5,I) =            0.5*(TX2-TX1)
        AA(6,6,I) =      -1.0
        AA(6,7,I) =            0.5           *DXII
        AA(6,12,I)=                + MZ_WZ(I)*DXII * 0.5
        CC(6,1,I) =  (MY2+MY1)*0.5
        CC(6,2,I) = -(MX2+MX1)*0.5 + MZ_TY(I)*DXII * 0.5
        CC(6,3,I) =                  MZ_TX(I)*DXII * 0.5
        CC(6,4,I) =           -0.5*(TY2-TY1)
        CC(6,5,I) =            0.5*(TX2-TX1)
        CC(6,6,I) =   1.0
        CC(6,7,I) =            0.5           *DXII
        CC(6,12,I)=                + MZ_WZ(I)*DXII * 0.5
C
C
C------ x-shear
        RR(7,I)   =   SX2-SX1
     &             + (SY2+SY1)*0.5*(TZ2-TZ1)
     &             + (SZ2+SZ1)*0.5*(TY2-TY1) - PX(I)    * DXII
C
        AA(7,2,I) = -(SZ2+SZ1)*0.5           - PX_TY(I) * DXII * 0.5
        AA(7,3,I) = -(SY2+SY1)*0.5           - PX_TZ(I) * DXII * 0.5
        AA(7,7,I) =      -1.0
        AA(7,8,I) =            0.5*(TZ2-TZ1)
        AA(7,9,I) =            0.5*(TY2-TY1)
        CC(7,2,I) =  (SZ2+SZ1)*0.5           - PX_TY(I) * DXII * 0.5
        CC(7,3,I) =  (SY2+SY1)*0.5           - PX_TZ(I) * DXII * 0.5
        CC(7,7,I) =   1.0
        CC(7,8,I) =            0.5*(TZ2-TZ1)
        CC(7,9,I) =            0.5*(TY2-TY1)
C
C------ y-shear
        RR(8,I)   =   SY2-SY1
     &             - (SX2+SX1)*0.5*(TZ2-TZ1)
     &             + (SZ2+SZ1)*0.5*(TX2-TX1) + PY(I)    * DXII
        AA(8,1,I) =  (SZ2+SZ1)*0.5
        AA(8,3,I) = -(SX2+SX1)*0.5
        AA(8,7,I) =           -0.5*(TZ2-TZ1)
        AA(8,8,I) =      -1.0
        AA(8,9,I) =            0.5*(TX2-TX1)
        CC(8,1,I) = -(SZ2+SZ1)*0.5
        CC(8,3,I) =  (SX2+SX1)*0.5
        CC(8,7,I) =           -0.5*(TZ2-TZ1)
        CC(8,8,I) =   1.0
        CC(8,9,I) =            0.5*(TX2-TX1)
C
C------ z-shear
        RR(9,I)   =   SZ2-SZ1
     &             - (SY2+SY1)*0.5*(TX2-TX1)
     &             - (SX2+SX1)*0.5*(TY2-TY1) - PZ(I)    * DXII
C
        AA(9,1,I) =  (SY2+SY1)*0.5           - PZ_TX(I) * DXII * 0.5
        AA(9,2,I) =  (SX2+SX1)*0.5           - PZ_TY(I) * DXII * 0.5
        AA(9,7,I) =           -0.5*(TY2-TY1)
        AA(9,8,I) =           -0.5*(TX2-TX1)
        AA(9,9,I) =      -1.0
        AA(9,12,I)=                          - PZ_WZ(I) * DXII * 0.5
        CC(9,1,I) = -(SY2+SY1)*0.5           - PZ_TX(I) * DXII * 0.5
        CC(9,2,I) = -(SX2+SX1)*0.5           - PZ_TY(I) * DXII * 0.5
        CC(9,7,I) =           -0.5*(TY2-TY1)
        CC(9,8,I) =           -0.5*(TX2-TX1)
        CC(9,9,I) =   1.0
        CC(9,12,I)=                          - PZ_WZ(I) * DXII * 0.5
C
C
        SB = SIN(BETA(I))
        CB = COS(BETA(I))
C
        IF(LSTRUC) THEN
         GJ = GJB(I) / EIREF
         EK = EKB(I) / EKREF
         EA = EAB(I) / EAREF
C
         EIZZ   = ( EIXXB(I)*CB**2 + EIYYB(I)*SB**2 ) / EIREF
         EIXX   = ( EIXXB(I)*SB**2 + EIYYB(I)*CB**2 ) / EIREF
         EIXZ   = ( EIXXB(I)*SB*CB - EIYYB(I)*SB*CB ) / EIREF
C
         EIZZ_B = (-EIXXB(I) + EIYYB(I) )* 2.0*SB*CB    / EIREF
         EIXX_B = ( EIXXB(I) - EIYYB(I) )* 2.0*SB*CB    / EIREF
         EIXZ_B = ( EIXXB(I) - EIYYB(I) )*(CB*CB-SB*SB) / EIREF
        ELSE
         EIBIG = 1.0E+8
C
         GJ = EIBIG
         EK = 0.0
         EA = EIBIG
C
         EIZZ = EIBIG
         EIXX = EIBIG
         EIXZ = 0.0
C
         EIZZ_B = 0.0
         EIXX_B = 0.0
         EIXZ_B = 0.0
        ENDIF
C
        MOMXD = (MOMX(I)+MOMX(I+1))*0.5 * DXII
        MOMYD = (MOMY(I)+MOMY(I+1))*0.5 * DXII
        MOMZD = (MOMZ(I)+MOMZ(I+1))*0.5 * DXII
C
C
C------ x-deflection angle
        RR(1,I+1)   =  EIZZ  *(TX2-TX1)
     &               - EIXZ  *(TZ2-TZ1) - (MX2+MX1)*0.5*DXII
C
        BB(1,1,I+1) = -EIZZ
        BB(1,2,I+1) =  EIZZ_B*(TX2-TX1) * 0.5
     &               - EIXZ_B*(TZ2-TZ1) * 0.5
        BB(1,3,I+1) =  EIXZ
        BB(1,4,I+1) =                              -0.5*DXII
        AA(1,1,I+1) =  EIZZ
        AA(1,2,I+1) =  EIZZ_B*(TX2-TX1) * 0.5
     &               - EIXZ_B*(TZ2-TZ1) * 0.5
        AA(1,3,I+1) = -EIXZ
        AA(1,4,I+1) =                              -0.5*DXII
C
C------ y-deflection angle (twist)
        RR(2,I+1)   =  GJ*(TY2-TY1)     - (MY2+MY1)*0.5*DXII
     &              -  EK*(WY2-WY1)
C
        BB(2,2,I+1) = -GJ
        BB(2,5,I+1) =                              -0.5*DXII
        BB(2,11,I+1)=  EK
        AA(2,2,I+1) =  GJ
        AA(2,5,I+1) =                              -0.5*DXII
        AA(2,11,I+1)= -EK
C
C------ z-deflection angle
        RR(3,I+1)   =  EIXX  *(TZ2-TZ1)
     &               - EIXZ  *(TX2-TX1) - (MZ2+MZ1)*0.5*DXII
C
        BB(3,1,I+1) =  EIXZ
        BB(3,2,I+1) =  EIXX_B*(TZ2-TZ1)
     &               - EIXZ_B*(TX2-TX1)
        BB(3,3,I+1) = -EIXX
        BB(3,6,I+1) =                              -0.5*DXII
        AA(3,1,I+1) = -EIXZ
        AA(3,2,I+1) =  EIXX_B*(TZ2-TZ1)
     &               - EIXZ_B*(TX2-TX1)
        AA(3,3,I+1) =  EIXX
        AA(3,6,I+1) =                              -0.5*DXII
C
C
C------ x-deflection
        RR(10,I+1)    = -WX2 + WX1 + (TZ2+TZ1)*0.5*DXII
        BB(10,3 ,I+1) =                        0.5*DXII
        BB(10,10,I+1) =        1.0
        AA(10,3 ,I+1) =                        0.5*DXII
        AA(10,10,I+1) = -1.0
C
C------ y-deflection
        RR(11,I+1)    = -WY2 + WY1 + (SY2+SY1)*0.5*DXII / EA
        BB(11,5 ,I+1) =                        0.5*DXII / EA
        BB(11,11,I+1) =        1.0
        AA(11,5 ,I+1) =                        0.5*DXII / EA
        AA(11,11,I+1) = -1.0
C
C------ z-deflection
        RR(12,I+1)    = -WZ2 + WZ1 + (TX2+TX1)*0.5*DXII
        BB(12,1 ,I+1) =                        0.5*DXII
        BB(12,12,I+1) =        1.0
        AA(12,1 ,I+1) =                        0.5*DXII
        AA(12,12,I+1) = -1.0
C
 10   CONTINUE
C
C---- set tip  M,S  to zero
      I = II+1
      AA(4,4,I) = 1.0
      AA(5,5,I) = 1.0
      AA(6,6,I) = 1.0
      AA(7,7,I) = 1.0
      AA(8,8,I) = 1.0
      AA(9,9,I) = 1.0
C
C
      CALL B12SOL(AA,BB,CC,RR,II+1)
C
c      do 5 i=1, ii+1
c        write(*,6666) i, (rr(k,i),k=1, 9)
c 6666   format(1x,i2,9f8.3)
c 5    continue
c
      RMAX = 0.0
      RMS = 0.0
C
C---- set under-relaxation factors
      DO 16 K=1, 12
        RLXR(K) = 1.0
 16   CONTINUE
C
      DO 18 I=1, II+1
        DO 181 K=1, 12
          IF(RLXR(K)*RR(K,I) .GT.  RRLIM(K)) RLXR(K) =  RRLIM(K)/RR(K,I)
          IF(RLXR(K)*RR(K,I) .LT. -RRLIM(K)) RLXR(K) = -RRLIM(K)/RR(K,I)
C
          RMAX = MAX( RMAX , ABS(RR(K,I)/RRLIM(K)) )
          RMS = RMS + (RR(K,I)/RRLIM(K))**2
 181    CONTINUE
 18   CONTINUE
C
      RMS = SQRT( RMS / FLOAT(9*II) )
C
C---- set minimum under-relaxation factor over all variables
      RLX = 1.0
      DO 19 K=1, 12
        RLX = AMIN1( RLX , RLXR(K) )
 19   CONTINUE
C
C---- update solution
      DO 20 I=1, II+1
        TX(I)   = TX(I)   - RLX*RR(1,I)
        TY(I)   = TY(I)   - RLX*RR(2,I)
        TZ(I)   = TZ(I)   - RLX*RR(3,I)
        MOMX(I) = MOMX(I) - RLX*RR(4,I)
        MOMY(I) = MOMY(I) - RLX*RR(5,I)
        MOMZ(I) = MOMZ(I) - RLX*RR(6,I)
        SHRX(I) = SHRX(I) - RLX*RR(7,I)
        SHRY(I) = SHRY(I) - RLX*RR(8,I)
        SHRZ(I) = SHRZ(I) - RLX*RR(9,I)
        WX(I)   = WX(I)   - RLX*RR(10,I)
        WY(I)   = WY(I)   - RLX*RR(11,I)
        WZ(I)   = WZ(I)   - RLX*RR(12,I)
c        WRITE(*,*)
c        WRITE(*,*) I
c        WRITE(*,1200) (RR(K,I),K=1,12)
c 1200   FORMAT( 4(1X, 3E12.4 /) )
 20   CONTINUE
C
C
cc      WRITE(*,1250) (RLXR(K), K=1, 12)
cc 1250 FORMAT(1X, 11F8.3)
C
      WRITE(*,1800) ITER, RMAX, RMS, RLX
 1800 FORMAT(1X,I3,'   max:',E9.3,'   rms:',E9.3,'   RLX =', F7.4)
C
      IF(RMAX .LE. EPS) GO TO 101
C
 100  CONTINUE
      WRITE(*,*) 'STCALC: Convergence failed.  Continuing ...'
C
 101  CONTINUE
C
C---- integrate towards tip for X displacements
c      I = 1
c      WX(I) = 0.0
c      DO 40 I=1, II
c        WX(I+1) =  WX(I)  -  (  TZ(I) +   TZ(I+1))*0.5 * DXII
c 40   CONTINUE
C
      RETURN
      END ! STCALC



      SUBROUTINE STADD
      INCLUDE 'XROTOR.INC'
C------------------------------------------------------
C     Adds on structural twist to static blade angles
C------------------------------------------------------
C
      DO 10 I=1, II
        BETA(I) = BETA0(I) + (TY(I) + TY(I+1))*0.5
 10   CONTINUE
C
      WRITE(*,1000) (BETA(II)-BETA0(II))*180.0/PI
C
      CONV = .FALSE.
      RETURN
C
 1000 FORMAT(/' New working blade angles set.'
     &       /' Tip angle deflection =', F8.3,'  deg.')
      END ! STADD


      SUBROUTINE STSET
      INCLUDE 'XROTOR.INC'
C------------------------------------------------------
C     Removes structural twist to get static blade angles
C------------------------------------------------------
C
      DO 10 I=1, II
        BETA0(I) = BETA(I) - (TY(I) + TY(I+1))*0.5
 10   CONTINUE
C
      WRITE(*,1000) (BETA(II)-BETA0(II))*180.0/PI
C
      CONV = .FALSE.
      RETURN
C
 1000 FORMAT(/' New static blade angles set.'
     &       /' Tip angle deflection =', F8.3,'  deg.')
      END ! STSET



      SUBROUTINE STWRIT(LU)
      INCLUDE 'XROTOR.INC'
C---------------------------------------------
C     Dumps blade force output to unit LU
C---------------------------------------------
C
      RTD = 180.0/PI
C
      IADD = 1
      IF(LU.EQ.LUWRIT) IADD = INCR
C
      WRITE(LU,1020)
C
      MOMREF = RHO*VEL**2 * RAD**3
C
C--- Deflections, moments and forces on blade beam
      DO 10 I=1, II, IADD
C
C********* still need to be averaged to i+1/2
C
C--- deflections
        WXA = WX(I)
        WYA = WY(I)
        WZA = WZ(I)
C--- angular deflections (deg)
        TXA = TX(I) * RTD
        TYA = TY(I) * RTD
        TZA = TZ(I) * RTD
C--- bending moments
        MXA = MOMX(I) * RHO * VEL**2 * RAD**3
        MYA = MOMY(I) * RHO * VEL**2 * RAD**3
        MZA = MOMZ(I) * RHO * VEL**2 * RAD**3
C--- shear forces
        SXA = SHRX(I) * RHO * VEL**2 * RAD**2
        SYA = SHRY(I) * RHO * VEL**2 * RAD**2
        SZA = SHRZ(I) * RHO * VEL**2 * RAD**2
C
        WRITE(LU,1035) I,XI(I),WXA,WZA,TYA,MZA,MXA,MYA,SYA,SXA,SZA        
   10 CONTINUE
C
C....................................................................
C
 1020 FORMAT(
     & /'  i    r/R     u/R     w/R     t         Mz          Mx'
     &  '           T            P            Sx           Sz' 
     & /'                             (deg)      (N-m)       (N-m)'
     &  '       (N-m)         (N)          (N)          (N)') 
C
 1035 FORMAT(1X,
     &    I2, F7.3, F8.4,F8.4, F7.2, 6(1X,G12.4) )
C
cc  i    r/R     u/R     w/R     t         Mz          Mx           T            P            Sx           Sz 
cc                            (deg)      (N-m)       (N-m)       (N-m)         (N)          (N)          (N)
cc  1  0.307  0.0000  0.0000   0.00   0.1338       0.5678E-01  -0.1882        529.7        3.883       -1.804    
ccXiifffffffFFFFFFFFffffffffFFFFFFFXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGGXGGGGGGGGGGGG
C
C
C--- Display the strain components on the blade beam
      COSR = COS(RAKE)
      WRITE(LU,2020)
C
      DO 20 I=1, II, IADD
C
        RST = RSTB(I) / RAD
        DXII = DXI(I)/COSR
C
C------ bending strains
        EX =  RST*(TZ(I+1)-TZ(I))/DXII * 1000.0
        EZ = -RST*(TX(I+1)-TX(I))/DXII * 1000.0
C
C------ extensional strain
        EY = (WY(I+1)-WY(I))/DXII * 1000.0
C------ torsional shear
        GT = RST*(TY(I+1)-TY(I))/DXII * 1000.0
C------ max normal strain
        EMAX = SQRT(EX**2 + EZ**2) + EY
C
        WRITE(LU,2030) I, XI(I), EX, EZ, EY, EMAX, GT
 20   CONTINUE
C
      RETURN

C
 2020 FORMAT(
     &  /' i    r/R      Ex       Ez       Ey      Emax'
     &   '      g     x 1000')
C          10  0.425   10.002   14.002   20.203   12.000   13.450
 2030 FORMAT(1X,
     &     I2, F7.3, 5F9.4 )
C
      END ! STWRIT



      SUBROUTINE B12SOL(A,B,C,R,II)
      DIMENSION A(12,12,II), B(12,12,II), C(12,12,II)
      DIMENSION R(12,1,II)
C-------------------------------------------------------
C      Solves the 12x12 block-tridiagonal Newton system
C      by a standard block elimination scheme.  
C      The solutions are returned in the R vectors.
C
C      |A C      ||d|   |R..|
C      |B A C    ||d|   |R..|
C      |  B . .  ||.| = |R..|
C      |    . . C||.|   |R..|
C      |      B A||d|   |R..|
C-------------------------------------------------------
C
      NRHS = 1
C
CCC** Forward sweep: Elimination of lower block diagonal (B's).
      DO 1 I=1, II
C
        IM = I-1
C
C------ don't eliminate first B block because it doesn't exist
        IF(I.EQ.1) GO TO 12
C
C------ eliminate Bi block, thus modifying Ai and Ci blocks
        DO 11 K=1, 12
          DO 111 L=1, 12
            A(K,L,I) = A(K,L,I)
     & - (  B(K, 1,I)*C( 1,L,IM)
     &    + B(K, 2,I)*C( 2,L,IM)
     &    + B(K, 3,I)*C( 3,L,IM)
     &    + B(K, 4,I)*C( 4,L,IM)
     &    + B(K, 5,I)*C( 5,L,IM)
     &    + B(K, 6,I)*C( 6,L,IM)
     &    + B(K, 7,I)*C( 7,L,IM)
     &    + B(K, 8,I)*C( 8,L,IM)
     &    + B(K, 9,I)*C( 9,L,IM)
     &    + B(K,10,I)*C(10,L,IM)
     &    + B(K,11,I)*C(11,L,IM)
     &    + B(K,12,I)*C(12,L,IM) )
  111     CONTINUE
          DO 112 L=1, NRHS
            R(K,L,I) = R(K,L,I)
     & - (  B(K, 1,I)*R( 1,L,IM)
     &    + B(K, 2,I)*R( 2,L,IM)
     &    + B(K, 3,I)*R( 3,L,IM)
     &    + B(K, 4,I)*R( 4,L,IM)
     &    + B(K, 5,I)*R( 5,L,IM)
     &    + B(K, 6,I)*R( 6,L,IM)
     &    + B(K, 7,I)*R( 7,L,IM)
     &    + B(K, 8,I)*R( 8,L,IM)
     &    + B(K, 9,I)*R( 9,L,IM)
     &    + B(K,10,I)*R(10,L,IM)
     &    + B(K,11,I)*R(11,L,IM)
     &    + B(K,12,I)*R(12,L,IM) )
  112     CONTINUE
   11   CONTINUE
C
C                                                              -1
CCC---- multiply Ci block and righthand side Ri vectors by (Ai)
C       using Gaussian elimination.
C
ccc        CALL SHOBLK(12,I,A(1,1,I))
C
   12   DO 13 KPIV=1, 11
          KP1 = KPIV+1
C
C-------- find max pivot index KX
          KX = KPIV
          DO 131 K=KP1, 12
            IF(ABS(A(K,KPIV,I))-ABS(A(KX,KPIV,I))) 131,131,1311
 1311        KX = K
  131     CONTINUE
C
          IF(A(KX,KPIV,I).EQ.0.0) THEN
           WRITE(*,*) 'Singular A block, i = ',I
           STOP
          ENDIF
C
          PIVOT = 1.0/A(KX,KPIV,I)
C
C-------- switch pivots
          A(KX,KPIV,I) = A(KPIV,KPIV,I)
C
C-------- switch rows & normalize pivot row
          DO 132 L=KP1, 12
            TEMP = A(KX,L,I)*PIVOT
            A(KX,L,I) = A(KPIV,L,I)
            A(KPIV,L,I) = TEMP
  132     CONTINUE
C
          DO 133 L=1, 12
            TEMP = C(KX,L,I)*PIVOT
            C(KX,L,I) = C(KPIV,L,I)
            C(KPIV,L,I) = TEMP
  133     CONTINUE
C
          DO 134 L=1, NRHS
            TEMP = R(KX,L,I)*PIVOT
            R(KX,L,I) = R(KPIV,L,I)
            R(KPIV,L,I) = TEMP
  134     CONTINUE
C
C-------- forward eliminate everything
          DO 135 K=KP1, 12
            ATMP = -A(K,KPIV,I)
            IF(ATMP.EQ.0.0) GO TO 135
            DO 1351 L=KP1, 12
              A(K,L,I) = A(K,L,I) + ATMP*A(KPIV,L,I)
 1351       CONTINUE
            C(K, 1,I) = C(K, 1,I) + ATMP*C(KPIV, 1,I)
            C(K, 2,I) = C(K, 2,I) + ATMP*C(KPIV, 2,I)
            C(K, 3,I) = C(K, 3,I) + ATMP*C(KPIV, 3,I)
            C(K, 4,I) = C(K, 4,I) + ATMP*C(KPIV, 4,I)
            C(K, 5,I) = C(K, 5,I) + ATMP*C(KPIV, 5,I)
            C(K, 6,I) = C(K, 6,I) + ATMP*C(KPIV, 6,I)
            C(K, 7,I) = C(K, 7,I) + ATMP*C(KPIV, 7,I)
            C(K, 8,I) = C(K, 8,I) + ATMP*C(KPIV, 8,I)
            C(K, 9,I) = C(K, 9,I) + ATMP*C(KPIV, 9,I)
            C(K,10,I) = C(K,10,I) + ATMP*C(KPIV,10,I)
            C(K,11,I) = C(K,11,I) + ATMP*C(KPIV,11,I)
            C(K,12,I) = C(K,12,I) + ATMP*C(KPIV,12,I)
            DO 1352 L=1, NRHS
              R(K,L,I) = R(K,L,I) + ATMP*R(KPIV,L,I)
 1352       CONTINUE
  135     CONTINUE
C
   13   CONTINUE
C
C------ solve for last row
        IF(A(12,12,I).EQ.0.0) THEN
         WRITE(*,*) 'Singular A block, i = ',I
         STOP
        ENDIF
        PIVOT = 1.0/A(12,12,I)
        C(12, 1,I) = C(12, 1,I)*PIVOT
        C(12, 2,I) = C(12, 2,I)*PIVOT
        C(12, 3,I) = C(12, 3,I)*PIVOT
        C(12, 4,I) = C(12, 4,I)*PIVOT
        C(12, 5,I) = C(12, 5,I)*PIVOT
        C(12, 6,I) = C(12, 6,I)*PIVOT
        C(12, 7,I) = C(12, 7,I)*PIVOT
        C(12, 8,I) = C(12, 8,I)*PIVOT
        C(12, 9,I) = C(12, 9,I)*PIVOT
        C(12,10,I) = C(12,10,I)*PIVOT
        C(12,11,I) = C(12,11,I)*PIVOT
        C(12,12,I) = C(12,12,I)*PIVOT
        DO 14 L=1, NRHS
          R(12,L,I) = R(12,L,I)*PIVOT
   14   CONTINUE
C
C------ back substitute everything
        DO 15 KPIV=10, 1, -1
          KP1 = KPIV+1
          DO 151 K=KP1, 12
            C(KPIV, 1,I) = C(KPIV, 1,I) - A(KPIV,K,I)*C(K, 1,I)
            C(KPIV, 2,I) = C(KPIV, 2,I) - A(KPIV,K,I)*C(K, 2,I)
            C(KPIV, 3,I) = C(KPIV, 3,I) - A(KPIV,K,I)*C(K, 3,I)
            C(KPIV, 4,I) = C(KPIV, 4,I) - A(KPIV,K,I)*C(K, 4,I)
            C(KPIV, 5,I) = C(KPIV, 5,I) - A(KPIV,K,I)*C(K, 5,I)
            C(KPIV, 6,I) = C(KPIV, 6,I) - A(KPIV,K,I)*C(K, 6,I)
            C(KPIV, 7,I) = C(KPIV, 7,I) - A(KPIV,K,I)*C(K, 7,I)
            C(KPIV, 8,I) = C(KPIV, 8,I) - A(KPIV,K,I)*C(K, 8,I)
            C(KPIV, 9,I) = C(KPIV, 9,I) - A(KPIV,K,I)*C(K, 9,I)
            C(KPIV,10,I) = C(KPIV,10,I) - A(KPIV,K,I)*C(K,10,I)
            C(KPIV,11,I) = C(KPIV,11,I) - A(KPIV,K,I)*C(K,11,I)
            C(KPIV,12,I) = C(KPIV,12,I) - A(KPIV,K,I)*C(K,12,I)
            DO 1511 L=1, NRHS
              R(KPIV,L,I) = R(KPIV,L,I) - A(KPIV,K,I)*R(K,L,I)
 1511       CONTINUE
  151     CONTINUE
   15   CONTINUE
    1 CONTINUE
C
CCC** Backward sweep: Back substitution using upper block diagonal (Ci's).
      DO 2 I=II-1, 1, -1
        IP = I+1
        DO 21 L=1, NRHS
          DO 211 K=1, 12
            R(K,L,I) = R(K,L,I)
     & - (  R( 1,L,IP)*C(K, 1,I)
     &    + R( 2,L,IP)*C(K, 2,I)
     &    + R( 3,L,IP)*C(K, 3,I)
     &    + R( 4,L,IP)*C(K, 4,I)
     &    + R( 5,L,IP)*C(K, 5,I)
     &    + R( 6,L,IP)*C(K, 6,I)
     &    + R( 7,L,IP)*C(K, 7,I)
     &    + R( 8,L,IP)*C(K, 8,I)
     &    + R( 9,L,IP)*C(K, 9,I)
     &    + R(10,L,IP)*C(K,10,I)
     &    + R(11,L,IP)*C(K,11,I) 
     &    + R(12,L,IP)*C(K,12,I) )
  211     CONTINUE
   21   CONTINUE
    2 CONTINUE
C
      RETURN
      END ! B12SOL



      SUBROUTINE SHOBLK(N,I,A)
      REAL A(N,N)
C
      WRITE(*,*)
      WRITE(*,*) 'i = ',I
      DO 10 K=1, N
        WRITE(*,1000) (A(K,L),L=1, 12)
 1000   FORMAT(1X,12F13.3 / )
   10 CONTINUE
      RETURN
      END



      SUBROUTINE STPLOT
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 DUMMY
C
      DIMENSION EX(IX),EY(IX),EZ(IX),GT(IX),EM(IX)
      DIMENSION BMX(IX),BMY(IX),BMZ(IX),SFY(IX)
      EQUIVALENCE (EX(1),W1(1)),
     &            (EY(1),W2(1)),
     &            (EZ(1),W3(1)),
     &            (GT(1),W4(1)),
     &            (EM(1),W5(1))
      EQUIVALENCE (BMX(1),W6(1)),
     &            (BMY(1),W7(1)),
     &            (BMZ(1),W8(1)),
     &            (SFY(1),W9(1))
C
      PLFAC  = 0.85
      PLFACD = 0.70
      STPAR  = 0.60
C
      MOMREF = RHO*VEL**2 * RAD**3
      FORREF = RHO*VEL**2 * RAD**2
C
      COSR = COS(RAKE)
      DO 10 I=1, II
C
        DXII = DXI(I)/COSR
C
        RST = RSTB(I) / RAD
C
C------ bending strains  (Ez is actually negated here)
        EX(I) =  RST*(TZ(I+1)-TZ(I))/DXII * 1000.0
        EZ(I) =  RST*(TX(I+1)-TX(I))/DXII * 1000.0
C
C------ extensional strain
        EY(I) = (WY(I+1)-WY(I))/DXII * 1000.0
C
C------ torsional shear (negated)
        GT(I) = -RST*(TY(I+1)-TY(I))/DXII * 1000.0
C
C------ max normal strain
        EM(I) = SQRT(EX(I)**2 + EZ(I)**2) + EY(I)
C
C
        BMX(I) =  MOMX(I) * MOMREF
        BMY(I) = -MOMY(I) * MOMREF
        BMZ(I) =  MOMZ(I) * MOMREF
        SFY(I) =  SHRY(I) * FORREF
 10   CONTINUE
C
      MXMAX = 0.0
      MYMAX = 0.0
      MZMAX = 0.0
      SYMAX = 0.0
      DO 20 I=1, II
        MXMAX = MAX( MXMAX , ABS(BMX(I)) )
        MYMAX = MAX( MYMAX , ABS(BMY(I)) )
        MZMAX = MAX( MZMAX , ABS(BMZ(I)) )
        SYMAX = MAX( SYMAX , ABS(SFY(I)) )
 20   CONTINUE
C
      IF(MXMAX .GT .0.0) XMX = AINT( LOG10(MXMAX) + 100.0 ) - 99.0
      IF(MYMAX .GT .0.0) XMY = AINT( LOG10(MYMAX) + 100.0 ) - 99.0
      IF(MZMAX .GT .0.0) XMZ = AINT( LOG10(MZMAX) + 100.0 ) - 99.0
      IF(SYMAX .GT .0.0) XSY = AINT( LOG10(SYMAX) + 100.0 ) - 99.0
C
      SFR = MAX( 0.8 , ABS(SCRNFR) )
      CALL PLTINI(SFR,IPSLU,IDEV,PLFACD*SIZE,LPLOT,.NOT.LLAND)
C
      CALL PLOTABS(1.0,0.75,-3)
C
      CALL PLEP(II,XI(1),EX,EY,EZ,GT,EM,
     &          1.0,0.2, 2.0,0.5, STPAR,1.2*CSIZE )
C
      CALL PLOT(0.0,STPAR+10.0*CSIZE,-3)
C
      CALL PLMF(II,XI(1),BMX,BMY,BMZ,SFY,
     &          1.0,0.2, 1.0,0.2, STPAR,1.2*CSIZE,
     &          XMX,XMY,XMZ,XSY )
      CALL PLFLUSH
ccc      CALL ASKC1('Hit <return>^',DUMMY)
      RETURN
      END ! STPLOT


      SUBROUTINE PLEP(N,X,EX,EY,EZ,GT,EM,
     &                XMAX,DX, YMAX,DY, PAR, CH)
      IMPLICIT REAL (M)
      DIMENSION X(N),EX(N),EY(N),EZ(N),GT(N),EM(N)
      EXTERNAL PLCHAR
C
      DIMENSION XLIN(2), YLIN(2)
      DATA XLIN / 0.0, 1.0 /
      DATA YLIN / 0.0, 0.0 /
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      RTD = 45.0/ATAN(1.0)
C
      XFAC  = 1.0/XMAX
      YFAC  = PAR/YMAX
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,1.0,DX*XFAC,0.0,DX,CH,1)
      CALL YAXIS(0.0,0.0,PAR,DY*YFAC,0.0,DY,CH,1)
ccc      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT(1.0/(XFAC*DX) + 0.01)
       NYG = INT(PAR/(YFAC*DY) + 0.01)
       CALL PLGRID(0.0,0.0, NXG,DX*XFAC, NYG,DY*YFAC, LMASK2 )
ccc      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(1.0-2.5*DX*XFAC-1.8*CH,-3.0*CH,1.2*CH,'r/R',0.0,3)
      CALL PLCHAR(0.0,PAR+1.0*CH,1.2*CH,'millistrain',0.0,11)
C
C
      CALL GETCOLOR(ICOL0)
C
      XLEN = 0.375*DX*XFAC
C
      XPL = (1.0 - 0.975*DX*XFAC)/XLEN
      YPL = (PAR - 2.0*CH       )/XLEN
C
      CALL NEWPEN(3)
      XPL1 = (XPL + 1.0)*XLEN + 1.0*CH
C
      CHL = 1.3*CH
C
      CALL NEWCOLORNAME('magenta')
      CALL XYLINE(N,X,EM,0.0,XFAC,0.0,YFAC,1)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,1)
      CALL PLMATH(XPL1,YPL*XLEN-0.5*CH,CHL,'e'  ,0.0,1)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CH,CHL,'max',0.0,3,PLCHAR)
C
      CALL NEWCOLORNAME('red')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,EX,0.0,XFAC,0.0,YFAC,2)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,2)
      CALL PLMATH(XPL1,YPL*XLEN-0.5*CH,CHL,'e'  ,0.0,1)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CH,CHL,'x'  ,0.0,1,PLCHAR)
C
      CALL NEWCOLORNAME('orange')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,EZ,0.0,XFAC,0.0,YFAC,3)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,3)
      CALL PLMATH(XPL1,YPL*XLEN-0.5*CH,CHL,'e'  ,0.0,1)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CH,CHL,'z'  ,0.0,1,PLCHAR)
C
      CALL NEWCOLORNAME('green')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,EY,0.0,XFAC,0.0,YFAC,4)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,4)
      CALL PLMATH(XPL1,YPL*XLEN-0.5*CH,CHL,'e'  ,0.0,1)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CH,CHL,'y'  ,0.0,1,PLCHAR)
C
      CALL NEWCOLORNAME('cyan')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,GT,0.0,XFAC,0.0,YFAC,5)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,5)
      CALL PLMATH(XPL1,YPL*XLEN-0.5*CH,CHL,'g'  ,0.0,1)
C
      CALL NEWCOLOR(ICOL0)
      RETURN
      END ! PLEP



      SUBROUTINE PLMF(N,X,MX,MY,MZ,SY,
     &                XMAX,DX, YMAX,DY, PAR, CH,
     &                XMX,XMY,XMZ,XSY )
      IMPLICIT REAL (M)
      DIMENSION X(N), MX(N),MY(N),MZ(N),SY(N)
      EXTERNAL PLCHAR
C
      DIMENSION XLIN(2), YLIN(2)
      DATA XLIN / 0.0, 1.0 /
      DATA YLIN / 0.0, 0.0 /
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      RTD = 45.0/ATAN(1.0)
C
      XFAC  = 1.0/XMAX
      YFAC  = PAR/YMAX
C
      MXFAC = PAR/(YMAX*10.0**XMX)
      MYFAC = PAR/(YMAX*10.0**XMY)
      MZFAC = PAR/(YMAX*10.0**XMZ)
      SYFAC = PAR/(YMAX*10.0**XSY)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,1.0,DX*XFAC,0.0,DX,CH,1)
      CALL YAXIS(0.0,0.0,PAR,DY*YFAC,0.0,DY,CH,1)
ccc      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT(1.0/(XFAC*DX) + 0.01)
       NYG = INT(PAR/(YFAC*DY) + 0.01)
       CALL PLGRID(0.0,0.0, NXG,DX*XFAC, NYG,DY*YFAC, LMASK2 )
ccc      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(1.0-2.5*DX*XFAC-1.8*CH,-3.0*CH,1.2*CH,'r/R',0.0,3)
      CALL PLCHAR(0.0,PAR+1.0*CH,1.2*CH,'Loads  (N-m, N)',0.0,15)
C
C
      CALL GETCOLOR(ICOL0)
C
      XLEN = 0.375*DX*XFAC
C
      XPL = (1.0 - 0.975*DX*XFAC)/XLEN
      YPL = (PAR - 2.0*CH       )/XLEN
C
      CHL = 1.1*CH
C
      CALL NEWPEN(3)
      XPL1 = (XPL + 1.0)*XLEN + 0.5*CHL
      XPL2 = (XPL + 1.0)*XLEN + 5.5*CHL
C
      CALL NEWCOLORNAME('red')
      CALL XYLINE(N,X,MX,0.0,XFAC,0.0,MXFAC,2)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,2)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CHL,CHL,'M /10',0.0,5)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CHL,CHL,'x'    ,0.0,1,PLCHAR)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CHL,0.7*CHL,XMX,0.0,-1)
C
      CALL NEWCOLORNAME('orange')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,MZ,0.0,XFAC,0.0,MZFAC,3)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,3)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CHL,CHL,'M /10',0.0,5)
      CALL PLSUBS(XPL1,YPL*XLEN-0.5*CHL,CHL,'z'    ,0.0,1,PLCHAR)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CHL,0.7*CHL,XMZ,0.0,-1)
C
      CALL NEWCOLORNAME('green')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,SY,0.0,XFAC,0.0,SYFAC,4)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,4)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CHL,CHL,'P /10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CHL,0.7*CHL,XSY,0.0,-1)
C
      CALL NEWCOLORNAME('cyan')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,MY,0.0,XFAC,0.0,MYFAC,5)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,5)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CHL,CHL,'T /10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CHL,0.7*CHL,XMY,0.0,-1)
C
      CALL NEWCOLOR(ICOL0)
      RETURN
      END ! PLMF


      SUBROUTINE EIPLOT
      INCLUDE 'XROTOR.INC'
ccc      CHARACTER*4 DUMMY
C
      PLFAC = 0.85
C
      EIMAX = 0.0
      EAMAX = 0.0
      GJMAX = 0.0
      EKMAX = 0.0
      MAMAX = 0.0
      DO 10 I=1, II
        EIMAX = MAX( EIMAX , ABS(EIYYB(I)) )
        EAMAX = MAX( EAMAX , ABS(EAB(I)  ) )
        GJMAX = MAX( GJMAX , ABS(GJB(I)  ) )
        EKMAX = MAX( EKMAX , ABS(EKB(I)  ) )
        MAMAX = MAX( MAMAX , ABS(MB(I)   ) )
 10   CONTINUE
C
      XEI = 0.
      XGJ = 0.
      XEK = 0.
      XEA = 0.
      XMA = 0.
C
      IF(EIMAX .GT .0.0) XEI = AINT( LOG10(EIMAX) + 100.95 ) - 100.0
      IF(GJMAX .GT .0.0) XGJ = AINT( LOG10(GJMAX) + 100.95 ) - 100.0
      IF(EKMAX .GT .0.0) XEK = AINT( LOG10(EKMAX) + 100.95 ) - 100.0
      IF(EAMAX .GT .0.0) XEA = AINT( LOG10(EAMAX) + 100.95 ) - 100.0
      IF(MAMAX .GT .0.0) XMA = AINT( LOG10(MAMAX) + 100.95 ) - 100.0
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
C
      CALL PLOTABS(1.0,1.75,-3)
      CALL PLEI(II,XI(1),RSTB, EIYYB,EAB,GJB,EKB,MB,
     &          1.0,0.2, 1.0,0.2, PAR,1.2*CSIZE,
     &          XEI, XGJ, XEK, XEA, XMA )
      CALL PLFLUSH
ccc      CALL ASKC1('Hit <return>^',DUMMY)
      RETURN
      END ! EIPLOT


      SUBROUTINE PLEI(N,X,R, EI,EA,GJ,EK,MA,
     &            XMAX,DX, YMAX,DY, PAR, CH,
     &            XEI, XGJ, XEK, XEA, XMA)
      IMPLICIT REAL (M)
      DIMENSION X(N),R(N)
      DIMENSION EI(N),EA(N),GJ(N),EK(N),MA(N)
C
      DIMENSION XLIN(2), YLIN(2)
      DATA XLIN / 0.0, 1.0 /
      DATA YLIN / 0.0, 0.0 /
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      RTD = 45.0/ATAN(1.0)
C
C
      XFAC  = 1.0/XMAX
      YFAC  = PAR/YMAX
C
      EIFAC = PAR/(YMAX*10.0**XEI)
      GJFAC = PAR/(YMAX*10.0**XGJ)
      EKFAC = PAR/(YMAX*10.0**XEK)
      EAFAC = PAR/(YMAX*10.0**XEA)
      MAFAC = PAR/(YMAX*10.0**XMA)
C
      CALL NEWPEN(2)
      CALL XAXIS(0.0,0.0,1.0,DX*XFAC,0.0,DX,CH,1)
      CALL YAXIS(0.0,0.0,PAR,DY*YFAC,0.0,DY,CH,1)
ccc      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT(1.0/(XFAC*DX) + 0.01)
       NYG = INT(PAR/(YFAC*DY) + 0.01)
       CALL PLGRID(0.0,0.0, NXG,DX*XFAC, NYG,DY*YFAC, LMASK2 )
ccc      ENDIF
C
      CALL NEWPEN(3)
      CALL PLCHAR(1.0-2.5*DX*XFAC-1.8*CH,-3.0*CH,1.2*CH,'r/R',0.0,3)
      CALL PLCHAR(0.0,PAR+1.0*CH,1.2*CH,'Mass, Stiffness (SI units)',
     &            0.0,15)
C
C
      XLEN = 0.375*DX*XFAC
C
      XPL = (1.0 - 0.95*DX*XFAC)/XLEN
      YPL = (PAR - 2.0*CH      )/XLEN
C
      XPL1 = (XPL + 1.0)*XLEN + 1.0*CH
      XPL2 = (XPL + 1.0)*XLEN + 6.0*CH
C
      CALL GETCOLOR(ICOL0)
C
      CALL NEWCOLORNAME('magenta')
      CALL XYLINE(N,X,EI,0.0,XFAC,0.0,EIFAC,1)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,1)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CH,CH,'EI/10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CH,0.7*CH,XEI,0.0,-1)
C
      CALL NEWCOLORNAME('red')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,GJ,0.0,XFAC,0.0,GJFAC,2)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,2)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CH,CH,'GJ/10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CH,0.7*CH,XGJ,0.0,-1)
C
      CALL NEWCOLORNAME('orange')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,EK,0.0,XFAC,0.0,EKFAC,3)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,3)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CH,CH,'EK/10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CH,0.7*CH,XEK,0.0,-1)
C
      CALL NEWCOLORNAME('yellow')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,EA,0.0,XFAC,0.0,EAFAC,4)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,4)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CH,CH,'EA/10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CH,0.7*CH,XEA,0.0,-1)
C
      CALL NEWCOLORNAME('cyan')
      YPL = YPL - 2.5*CH/XLEN
      CALL XYLINE(N,X,MA,0.0,XFAC,0.0,MAFAC,5)
      CALL XYLINE(2,XLIN,YLIN,-XPL,XLEN,-YPL,XLEN,5)
      CALL PLCHAR(XPL1,YPL*XLEN-0.5*CH,CH,'m /10',0.0,5)
      CALL PLNUMB(XPL2,YPL*XLEN+0.2*CH,0.7*CH,XMA,0.0,-1)
C
      CALL NEWCOLOR(ICOL0)
      RETURN
      END ! PLEI

