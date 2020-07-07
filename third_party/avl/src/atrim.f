C***********************************************************************
C    Module:  atrim.f
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


      SUBROUTINE TRMSET(COMAND,COMARG,LMATCH,IR)
      INCLUDE 'AVL.INC'
      CHARACTER*(*) COMAND, COMARG
      LOGICAL LMATCH
C
      CHARACTER*2 CNUM
C
      CHARACTER*4 COM
      CHARACTER*80 CARG, PROMPT, RTNEW
      LOGICAL ERROR, REPKEY
C
      INTEGER IINP(10)
      REAL RINP(10)
C
C---- for trim command, first number in arg string should be part of command
      IF(COMAND(1:1) .NE. 'C') THEN
       LMATCH = .FALSE.
       RETURN
      ENDIF
C
      CNUM = COMARG(1:2)
C
C
      IF    (CNUM.EQ.'1 ') THEN
       KTRIM = 1
cc       CALL TRIM1(IR)
       LMATCH = .TRUE.
C
      ELSEIF(CNUM.EQ.'2 ') THEN
       KTRIM = 2
cc       CALL TRIM2(IR)
       LMATCH = .TRUE.
C
      ELSE
       LMATCH = .FALSE.
       RETURN
C
      ENDIF
C
C
C---- start here with new run case
 100  CONTINUE
C
      IF(KTRIM.EQ.1 .OR.
     &   KTRIM.EQ.2      ) THEN
       IF(PARVAL(IPCL,IR) .EQ. 0.0) THEN
C------ current case has CL=0 ... set it using CL constraint, if it's present
        DO IV = 1, NVTOT
          IF(ICON(IV,IR) .EQ. ICCL) THEN
           WRITE(*,*)
           WRITE(*,*)'       Setting trim CL from current CL constraint'
           PARVAL(IPCL,IR) = CONVAL(ICCL,IR)
           GO TO 101
          ENDIF
        ENDDO
       ENDIF
 101   CONTINUE
      ENDIF
C
      WRITE(*,*)
C
C---- tag this run case with trim type (not converged yet)
      ITRIM(IR) = -KTRIM
C
      IF(PARVAL(IPRHO,IR) .LE. 0.0) PARVAL(IPRHO,IR)  = RHO0
      IF(PARVAL(IPGEE,IR) .LE. 0.0) PARVAL(IPGEE,IR)  = GEE0
      IF(PARVAL(IPMASS,IR).LE. 0.0) PARVAL(IPMASS,IR) = RMASS0
C
      IR1 = IR
      IR2 = IR
C
C----------------------------------------------------------------
C---- jump back here to calculate stuff depending on case
 5    CONTINUE
C
      CREFD = CREF*UNITL
      BREFD = BREF*UNITL
      SREFD = SREF*UNITL**2
C
C- - - - - - - - - - - - - - - - - - - - - - - - 
 7500 FORMAT('     .. setting new ', A,' for run case', I3)
C
      IF    (KTRIM.EQ.1) THEN
C------ Level or banked horizontal flight:    recalculate V or CL, Radius
C
        DO 7 JR = IR1, IR2
          PHI = PARVAL(IPPHI,JR)
          THE = PARVAL(IPTHE,JR)
          CL  = PARVAL(IPCL ,JR)
          CD0 = PARVAL(IPCD0,JR)
          VEE = PARVAL(IPVEE,JR)
          RAD = PARVAL(IPRAD,JR)
          RHO = PARVAL(IPRHO,JR)
          GEE = PARVAL(IPGEE,JR)
          FAC = PARVAL(IPFAC,JR)
          RMASS = PARVAL(IPMASS,JR)
C
          SINP = SIN(PHI*DTR)
          COSP = COS(PHI*DTR)
C
C-------- set velocity ?
          IF(VEE .LE. 0.0 .AND.
     &       CL  .GT. 0.0       ) THEN
           VEE = SQRT( 2.0*RMASS*GEE / (RHO*SREFD*CL*COSP) )
           PARVAL(IPVEE,JR) = VEE
           WRITE(*,7500) 'velocity', JR
          ENDIF
C
C-------- set CL ?
          IF(CL  .LE. 0.0 .AND.
     &       VEE .GT. 0.0       ) THEN
           CL  = 2.0*RMASS*GEE / (RHO*SREFD*VEE**2*COSP)
           PARVAL(IPCL,JR) = CL
           WRITE(*,7500) 'CL', JR
          ENDIF
C
          IF(SINP .EQ. 0.0) THEN
           RAD = 0.
          ELSE
           RAD = VEE**2 * COSP / (GEE * SINP)
          ENDIF
          PARVAL(IPRAD,JR) = RAD
          WRITE(*,7500) 'turn radius', JR
C
          FAC = 1.0/COSP
          PARVAL(IPFAC,JR) = FAC
          WRITE(*,7500) 'load factor', JR
C
          THE = 0.
          PARVAL(IPTHE,JR) = THE
cc        WRITE(*,7500) 'elevation', JR
C
C-------- set up CL and rotation rate constraints
          IF(RAD .GT. 0.0) THEN
           WHX = 0.
           WHY = SINP * CREFD/(2.0*RAD)
           WHZ = COSP * BREFD/(2.0*RAD)
          ELSE
           WHX = 0.
           WHY = 0.
           WHZ = 0.
          ENDIF
C
          CONVAL(ICCL  ,JR) = CL 
          CONVAL(ICROTX,JR) = WHX
          CONVAL(ICROTY,JR) = WHY
          CONVAL(ICROTZ,JR) = WHZ
C 
          ICON(IVALFA,JR) = ICCL
          ICON(IVROTX,JR) = ICROTX
          ICON(IVROTY,JR) = ICROTY
          ICON(IVROTZ,JR) = ICROTZ
 7      CONTINUE
C
C- - - - - - - - - - - - - - - - - - - - - - - - 
      ELSEIF(KTRIM.EQ.2) THEN
C------ Steady pitch rate (looping flight
C
        DO 8 JR = IR1, IR2
          PHI = PARVAL(IPPHI,JR)
          THE = PARVAL(IPTHE,JR)
          CL  = PARVAL(IPCL ,JR)
          CD0 = PARVAL(IPCD0,JR)
          VEE = PARVAL(IPVEE,JR)
          RAD = PARVAL(IPRAD,JR)
          RHO = PARVAL(IPRHO,JR)
          GEE = PARVAL(IPGEE,JR)
          FAC = PARVAL(IPFAC,JR)
          RMASS = PARVAL(IPMASS,JR)
C
          SINP = SIN(PHI*DTR)
          COSP = COS(PHI*DTR)
C
C-------- set radius ?
          IF(RAD .EQ. 0.0 .AND.
     &       CL  .GT. 0.0      ) THEN
           RAD = RMASS / (0.5*RHO * SREFD * CL)
           PARVAL(IPRAD,JR) = RAD
           WRITE(*,7500) 'turn radius', JR
          ENDIF
C
C-------- set CL ?
          IF(RAD .GT. 0.0 .AND.
     &       CL  .EQ. 0.0      ) THEN
           CL = RMASS / (0.5*RHO * SREFD * RAD)
           PARVAL(IPCL,JR) = CL
           WRITE(*,7500) 'CL', JR
          ENDIF
C
C-------- set load factor ?
          IF(FAC .EQ. 0.0 .AND.
     &       CL  .GT. 0.0 .AND. 
     &       VEE .GT. 0.0 .AND. 
     &       GEE .GT. 0.0      ) THEN
           FAC = 0.5*RHO*VEE**2 * SREFD * CL / (RMASS*GEE)
           PARVAL(IPFAC,JR) = FAC
           WRITE(*,7500) 'load factor', JR
          ENDIF
C
C-------- set velocity ?
          IF(FAC .GT. 0.0 .AND.
     &       CL  .GT. 0.0 .AND. 
     &       VEE .EQ. 0.0 .AND. 
     &       GEE .GT. 0.0      ) THEN
           VEE = SQRT( FAC * RMASS*GEE / (0.5*RHO*SREFD*CL) )
           PARVAL(IPVEE,JR) = VEE
           WRITE(*,7500) 'velocity', JR
          ENDIF
C
          THE = 0.
          PARVAL(IPTHE,JR) = THE
cc        WRITE(*,7500) 'elevation', JR
C
C-------- set up CL and rotation rate constraints
          IF(RAD .GT. 0.0) THEN
           WHX = 0.
           WHY = CREFD/(2.0*RAD)
           WHZ = 0.
          ELSE
           WHX = 0.
           WHY = 0.
           WHZ = 0.
          ENDIF
C
          CONVAL(ICCL  ,JR) = CL 
          CONVAL(ICROTX,JR) = WHX
          CONVAL(ICROTY,JR) = WHY
          CONVAL(ICROTZ,JR) = WHZ
C 
          ICON(IVALFA,JR) = ICCL
          ICON(IVROTX,JR) = ICROTX
          ICON(IVROTY,JR) = ICROTY
          ICON(IVROTZ,JR) = ICROTZ
 8      CONTINUE
C
      ENDIF
C
      PHI = PARVAL(IPPHI,IR)
      THE = PARVAL(IPTHE,IR)
      CL  = PARVAL(IPCL ,IR)
      CD0 = PARVAL(IPCD0,IR)
      VEE = PARVAL(IPVEE,IR)
      RAD = PARVAL(IPRAD,IR)
      RHO = PARVAL(IPRHO,IR)
      GEE = PARVAL(IPGEE,IR)
      FAC = PARVAL(IPFAC,IR)
      XCG = PARVAL(IPXCG,IR)
      YCG = PARVAL(IPYCG,IR)
      ZCG = PARVAL(IPZCG,IR)
      RMASS = PARVAL(IPMASS,IR)
C
      SINP = SIN(PHI*DTR)
      COSP = COS(PHI*DTR)
C
 1000 FORMAT(A)
 2000 FORMAT(/'     Setup of trimmed run case ',A,':  ', A)
 2005 FORMAT( '     ', A
     &       /'     =================================================')
 2105 FORMAT(6X, A, G10.4, 2X, A)
 2110 FORMAT(6X, A, A )
C
C--------------------------------------------------------------------------
C----- jump back here just for menu
 10    CONTINUE
       CALL CFRAC(IR,NRUN,PROMPT,NPR)
       WRITE(*,2000) PROMPT(1:NPR),RTITLE(IR)
C
       IF    (KTRIM.EQ.1) THEN
        WRITE(*,2005) '(level or banked horizontal flight)'
        WRITE(*,2105) 'B  bank angle = ', PHI  , 'deg'
        WRITE(*,2105) 'C  CL         = ', CL   , ' '
        WRITE(*,2105) 'V  velocity   = ', VEE  , UNCHV(1:NUV)
        WRITE(*,2105) 'M  mass       = ', RMASS, UNCHM(1:NUM)
        WRITE(*,2105) 'D  air dens.  = ', RHO  , UNCHD(1:NUD)
        WRITE(*,2105) 'G  grav.acc.  = ', GEE  , UNCHA(1:NUA)
        WRITE(*,2105) '   turn rad.  = ', RAD  , UNCHL(1:NUL)
        WRITE(*,2105) '   load fac.  = ', FAC  , ' '
        WRITE(*,2105) 'X  X_cg       = ', XCG  , 'Lunit'
        WRITE(*,2105) 'Y  Y_cg       = ', YCG  , 'Lunit'
        WRITE(*,2105) 'Z  Z_cg       = ', ZCG  , 'Lunit'
C
       ELSEIF(KTRIM.EQ.2) THEN
        WRITE(*,2005) '(steady pitch rate - looping flight)'
        WRITE(*,2105) 'C  CL        = ', CL   , ' '
        WRITE(*,2105) 'V  velocity  = ', VEE  , UNCHV(1:NUV) 
        WRITE(*,2105) 'M  mass      = ', RMASS, UNCHM(1:NUM)
        WRITE(*,2105) 'D  air dens. = ', RHO  , UNCHD(1:NUD) 
        WRITE(*,2105) 'G  grav.acc. = ', GEE  , UNCHA(1:NUA) 
        WRITE(*,2105) 'R  turn rad. = ', RAD  , UNCHL(1:NUL) 
        WRITE(*,2105) 'L  load fac. = ', FAC  , ' '
        WRITE(*,2105) 'X  X_cg      = ', XCG  , 'Lunit'
        WRITE(*,2105) 'Y  Y_cg      = ', YCG  , 'Lunit'
        WRITE(*,2105) 'Z  Z_cg      = ', ZCG  , 'Lunit'
C
       ENDIF
C
       CALL ASKC('     Enter parameter, value  (or  # - + N )',COM,CARG)
C
       IF(COM.EQ.'    ') THEN
C------ just a Return entered... go back
        RETURN
       ENDIF
C
C
C------------------------------------------------------
C---- check for run case commands
      IF(COM.EQ.'+   ') THEN
C----- add new case after current one
C
       IF(NRUN.EQ.NRMAX) THEN
        WRITE(*,*)
        WRITE(*,*) '* Run case array limit NRMAX reached'
       ELSE
        NRUN = NRUN + 1
C
        DO JR = NRUN, IR+1, -1
          CALL RCOPY(JR,JR-1)
        ENDDO
        WRITE(*,*) 'Initializing new run case from current one'
C        
        IR = IR + 1
       ENDIF
C
       GO TO 100
C
      ELSEIF(COM.EQ.'-   ') THEN
C----- delete current case
C
       IF(NRUN.LE.1) THEN
        WRITE(*,*)
        WRITE(*,*) '* Cannot delete one remaining run case'
       ELSE
        DO JR = IR, NRUN-1
          CALL RCOPY(JR,JR+1)
        ENDDO
        NRUN = NRUN - 1
        IRUN = MAX( 1 , MIN( IRUN , NRUN ) )
       ENDIF
C
       GO TO 100
C
      ENDIF
C
C------------------------------------------------------
C---- see if command is an integer (new run case selection)
      NINP = 1
      CALL GETINT(COM,IINP,NINP,ERROR)
      IF(.NOT.ERROR .AND. NINP.GE.1 
     &  .AND. COM(1:1).NE.'T'
     &  .AND. COM(1:1).NE.'F' ) THEN
C----- command is an integer... new case index?
       IF(IINP(1).LT.1 .OR. IINP(1).GT.NRUN) THEN
        WRITE(*,*)
        WRITE(*,*) '* Selected new run case is not defined'
        GO TO 10
       ELSE
C------ valid new run case selected... go back to top of menu
        IR = IINP(1)
        GO TO 100
       ENDIF
      ENDIF
C
C
C------------------------------------------------------
C---- extract argument, if any
      NINP = 1
      CALL GETFLT(CARG,RINP,NINP,ERROR)
C
C
      IF    (COM(1:1) .EQ. COM(2:2)) THEN
       REPKEY = .TRUE.
       IR1 = 1
       IR2 = NRUN
       COM(2:2) = ' '
      ELSEIF(COM(1:2) .EQ. COM(3:4)) THEN
       REPKEY = .TRUE.
       IR1 = 1
       IR2 = NRUN
       COM(3:4) = '  '
      ELSE
       REPKEY = .FALSE.
       IR1 = IR
       IR2 = IR
      ENDIF
C
C
C==============================================================================
C---- now decode regular parameter value commands
C
C------------------------------------
      IF(COM(1:1).EQ.'B') THEN
        IF    (KTRIM.EQ.1) THEN
 11      CONTINUE
         IF(NINP.GE.1) THEN
          PHI = RINP(1)
         ELSE
          CALL ASKR('      Enter bank angle^',PHI)
         ENDIF
C
         IF(PHI.LE.-90.0 .OR. PHI.GE.90.0) THEN
          WRITE(*,*) '    * Must have  -90 < bank < +90'
          NINP = 0
          GO TO 11
         ENDIF
C
         DO JR = IR1, IR2
           PARVAL(IPPHI,JR) = PHI
C
C--------- recalculate V, R, and N
           PARVAL(IPVEE,JR) = 0.
         ENDDO
C
        ELSEIF(KTRIM.EQ.2) THEN
         WRITE(*,*) 'Bank angle not used for this trim case'
         GO TO 10
C
        ENDIF
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'C' .OR.
     &       COM(1:2).EQ.'CL'      ) THEN
 21     CONTINUE
        IF(NINP.GE.1) THEN
         CL = RINP(1)
        ELSE
         CALL ASKR('      Enter CL^',CL)
        ENDIF
C
        IF(CL .LE. 0.0) THEN
         WRITE(*,*) '    * Must have  CL > 0'
         NINP = 0
         GO TO 21
        ENDIF

        DO JR = IR1, IR2
          PARVAL(IPCL,JR) = CL
C
          IF    (KTRIM.EQ.1) THEN
C--------- go recalculate V and R
           PARVAL(IPVEE,JR) = 0.
           PARVAL(IPRAD,JR) = 0.
C
          ELSEIF(KTRIM.EQ.2) THEN
C--------- go recalculate R and N
           PARVAL(IPRAD,JR) = 0.
           PARVAL(IPFAC,JR) = 0.
C
          ENDIF
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'V') THEN
 31     CONTINUE
        IF(NINP.GE.1) THEN
         VEE = RINP(1)
        ELSE
         CALL ASKR('      Enter velocity^',VEE)
        ENDIF
C
        IF(VEE.LE.0.0) THEN
         WRITE(*,*) '    * Must have  V > 0'
         NINP = 0
         GO TO 31
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPVEE,JR) = VEE
C
          IF    (KTRIM.EQ.1) THEN
C--------- go recalculate CL
           PARVAL(IPCL,JR) = 0.
C
          ELSEIF(KTRIM.EQ.2) THEN
C--------- go recalculate N
           PARVAL(IPFAC,JR) = 0.
C
          ENDIF
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'M') THEN
 41     CONTINUE
        IF(NINP.GE.1) THEN
         RMASS = RINP(1)
        ELSE
         CALL ASKR('      Enter mass^',RMASS)
        ENDIF
C
        IF(RMASS.LE.0.0) THEN
         WRITE(*,*) '    * Must have  m > 0'
         NINP = 0
         GO TO 41
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPMASS,JR) = RMASS
C
          IF    (KTRIM.EQ.1) THEN
C--------- go recalculate V
           PARVAL(IPVEE,JR) = 0.
C
          ELSEIF(KTRIM.EQ.2) THEN
C--------- go recalculate R and N
           PARVAL(IPRAD,JR) = 0.
           PARVAL(IPFAC,JR) = 0.
C
          ENDIF
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'D') THEN
 51     CONTINUE
        IF(NINP.GE.1) THEN
         RHO = RINP(1)
        ELSE
         CALL ASKR('      Enter air density^',RHO)
        ENDIF
C
        IF(RHO.LE.0.0) THEN
         WRITE(*,*) '    * Must have  rho > 0'
         NINP = 0
         GO TO 51
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPRHO,JR) = RHO
C
          IF    (KTRIM.EQ.1) THEN
C--------- go recalculate V
           PARVAL(IPVEE,JR) = 0.
C
          ELSEIF(KTRIM.EQ.2) THEN
C--------- go recalculate R and N
           PARVAL(IPRAD,JR) = 0.
           PARVAL(IPFAC,JR) = 0.
C
          ENDIF
        ENDDO

C-------------------------------------
      ELSEIF(COM(1:1).EQ.'G') THEN
 61     CONTINUE
        IF(NINP.GE.1) THEN
         GEE = RINP(1)
        ELSE
         CALL ASKR('      Enter gravity^',GEE)
        ENDIF
C
        IF(GEE.LE.0.0) THEN
         WRITE(*,*) '    * Must have  g > 0'
         NINP = 0
         GO TO 61
        ENDIF
C
        DO JR = IR1, IR2
          PARVAL(IPGEE,JR) = GEE
C
          IF    (KTRIM.EQ.1) THEN
C--------- go recalculate V
           PARVAL(IPVEE,JR) = 0.
C
          ELSEIF(KTRIM.EQ.2) THEN
C--------- go recalculate N
           PARVAL(IPFAC,JR) = 0.
C
          ENDIF
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'R') THEN
        IF    (KTRIM.EQ.1) THEN
         WRITE(*,*) 'Turn radius not specifiable for this trim case'
         GO TO 10
C
        ELSEIF(KTRIM.EQ.2) THEN
 71      CONTINUE
         IF(NINP.GE.1) THEN
          RAD = RINP(1)
         ELSE
          CALL ASKR('      Enter turn radius^',RAD)
         ENDIF
C
         IF(RAD.LE.0.0) THEN
          WRITE(*,*) '    * Must have  R > 0'
          NINP = 0
          GO TO 71
         ENDIF
C
         DO JR = IR1, IR2
           PARVAL(IPRAD,JR) = RAD
C
C--------- go recalculate CL, N
           PARVAL(IPCL ,JR) = 0.
           PARVAL(IPFAC,JR) = 0.
         ENDDO
C
        ENDIF
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'L') THEN
        IF    (KTRIM.EQ.1) THEN
         WRITE(*,*) 'Load factor not specifiable for this trim case'
         GO TO 10
C
        ELSEIF(KTRIM.EQ.2) THEN
 81      CONTINUE
         IF(NINP.GE.1) THEN
          FAC = RINP(1)
         ELSE
          CALL ASKR('      Enter load factor^',FAC)
         ENDIF
C
         IF(FAC.LE.0.0) THEN
          WRITE(*,*) '    * Must have  N > 0'
          NINP = 0
          GO TO 81
         ENDIF
C
         DO JR = IR1, IR2
           PARVAL(IPFAC,JR) = FAC
C
C--------- go recalculate V
           PARVAL(IPVEE,JR) = 0.
         ENDDO
C
        ENDIF
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'X') THEN
        IF(NINP.GE.1) THEN
         XCG = RINP(1)
        ELSE
         CALL ASKR('      Enter X_cg location^',XCG)
        ENDIF
        DO JR = IR1, IR2
          PARVAL(IPXCG,JR) = XCG
          WRITE(*,7500) 'x_cg', JR
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'Y') THEN
        IF(NINP.GE.1) THEN
         YCG = RINP(1)
        ELSE
         CALL ASKR('      Enter Y_cg location^',YCG)
        ENDIF
        DO JR = IR1, IR2
          PARVAL(IPYCG,JR) = YCG
          WRITE(*,7500) 'y_cg', JR
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'Z') THEN
        IF(NINP.GE.1) THEN
         ZCG = RINP(1)
        ELSE
         CALL ASKR('      Enter Z_cg location^',ZCG)
        ENDIF
        DO JR = IR1, IR2
          PARVAL(IPZCG,JR) = ZCG
          WRITE(*,7500) 'z_cg', JR
        ENDDO
C
C-------------------------------------
      ELSEIF(COM(1:1).EQ.'P') THEN
        IF(NINP.GE.1) THEN
         CD0 = RINP(1)
        ELSE
         CALL ASKR('      Enter profile CDo^',CD0)
        ENDIF
        DO JR = IR1, IR2
          PARVAL(IPCD0,JR) = CD0
          WRITE(*,7500) 'CDo', JR
        ENDDO
C
C------------------------------------------------------
      ELSEIF(COM .EQ. 'N') THEN
C------ change name of run case
        IF(CARG.NE.' ') THEN
         RTITLE(IR) = CARG
        ELSE
         WRITE(*,830) RTITLE(IR)
 830     FORMAT(/' Enter run case name:  ', A)
         READ(*,1000) RTNEW
         IF(RTNEW.NE.' ') RTITLE(IR) = RTNEW
        ENDIF
C
C-------------------------------------
      ELSE
        WRITE(*,*) '     * Unrecognized parameter'
        GO TO 10
C
      ENDIF
C
      GO TO 5
C
      END ! TRMSET

