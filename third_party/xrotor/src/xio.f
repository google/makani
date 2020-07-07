C***********************************************************************
C    Module:  xio.f
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

      SUBROUTINE SAVEOLD(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
C--------------------------------------------------------------------------
C     Save rotor and operating state in old XROTOR_Version < Ver6.9 format
C     The old format does not save more than one aero data set 
C     (even then it saves without altitude or the MCRIT or DCL_STALL 
C     parameters).
C--------------------------------------------------------------------------
      LOGICAL LVDUCT
C
      CHARACTER*1 ANS
C
      GREEK = .FALSE.
      IF(.NOT.CONV) THEN
       WRITE(*,1050)
       RETURN
      ENDIF
C
      LVDUCT = ABS(ADW-ADV*URDUCT) .GE. 5.E-5
C
      LU = LUTEMP
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter filename^',FNAME)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=5)
      WRITE(*,*)
      WRITE(*,*) 'Output file exists.  Overwrite?  Y'
      IF(INDEX('Nn',ANS).EQ.0) GO TO 6
C
      CLOSE(LU)
      WRITE(*,*) 'Current rotor not saved.'
      RETURN
C
 5    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=90)
 6    REWIND(LU)
C
      IF(NAME.EQ.' ') NAME = 'Saved blade'
C
      WRITE(LU,1100) NAME
      WRITE(LU,*) II,NBLDS
      WRITE(LU,*) RHO,VSO,RMU
      WRITE(LU,*) RAD,VEL,ADV
      WRITE(LU,*) XI0,XW0
C--- Save first aero section data
      N = 1
      CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      WRITE(LU,*) A0,DCLDA,CLMAX,CLMIN
      WRITE(LU,*) CDMIN,CLDMIN,DCDCL2
      WRITE(LU,*) REREF,REXP
C
      WRITE(LU,*) LVDUCT, DUCT, WIND
      DO I=1, II
        WRITE(LU,*) XI(I),CH(I),BETA0(I),UBODY(I)
      END DO
C
      WRITE(LU,*) URDUCT
C
      IF(NADD.GT.1) THEN
       DO I=1, NADD
         WRITE(LU,*) RADD(I), UADD(I), VADD(I)
       END DO
       WRITE(*,*) 'External slipstream included in save file'
      ENDIF
C
      CLOSE(LU)
      RETURN
C
 90   WRITE(*,*) 'Bad filename.'
      WRITE(*,*) 'Current rotor not saved.'
      RETURN
C
C...................................................................
 1000 FORMAT(A)
 1050 FORMAT(/' *** Converged operating solution does not exist ***')
 1100 FORMAT(A)
      END ! SAVEOLD



      SUBROUTINE SAVE(FNAME1)
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
C--------------------------------------------------------------------------
C     Save rotor and operating state in new XROTOR_Version 6.9 format
C     This format saves additional parameters including the aero data
C     sections.
C--------------------------------------------------------------------------
      LOGICAL LVDUCT
C
      CHARACTER*1 ANS
C
      GREEK = .FALSE.
cc      IF(.NOT.CONV) THEN
cc       WRITE(*,1050)
cc       RETURN
cc      ENDIF
C
      LVDUCT = ABS(ADW-ADV*URDUCT) .GE. 5.E-5
      LU = LUTEMP
      FNAME = FNAME1
C
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter filename^',FNAME)
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=5)
      WRITE(*,*)
      WRITE(*,*) 'Output file exists.  Overwrite?  Y'
      READ (*,1000) ANS
      IF(INDEX('Nn',ANS).EQ.0) GO TO 6
C
      CLOSE(LU)
      WRITE(*,*) 'Current rotor not saved.'
      RETURN
C
 5    OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=90)
 6    REWIND(LU)
C
C      
C--- Version header and case name
      IF(NAME.EQ.' ') NAME = 'Saved blade'
      WRITE(LU,1100) VERSION,NAME
C--- Altitude and atmospheric data
      WRITE(LU,1102) 
      WRITE(LU,1200) RHO,VSO,RMU,ALT
C--- Radius, velocity, advance ratio and blade rake angle
      WRITE(LU,1103)
      WRITE(LU,1200) RAD,VEL,ADV,RAKE
C
      WRITE(LU,1104)
      WRITE(LU,1200) XI0,XW0
C--- Save aero data for defined aero sections
      WRITE(LU,1105) NAERO
      MCRIT = 0.8
      DO N = 1, NAERO
       CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &              DCLDA,DCLDA_STALL,DCL_STALL,
     &              CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
       WRITE(LU,1106)
       WRITE(LU,1200) XISECT
       A0DEG = A0 *180.0/PI
       WRITE(LU,1107)
       WRITE(LU,1200) A0DEG,DCLDA,CLMAX,CLMIN
       WRITE(LU,1108) 
       WRITE(LU,1200) DCLDA_STALL,DCL_STALL,CMCON,MCRIT
       WRITE(LU,1109) 
       WRITE(LU,1200) CDMIN,CLDMIN,DCDCL2
       WRITE(LU,1110)
       WRITE(LU,1200) REREF,REXP
      END DO
C
C--- Save logical flags for duct and windmill 
      WRITE(LU,1111) LVDUCT, DUCT, WIND
C
C--- #radial stations and #blades
      WRITE(LU,1112) II,NBLDS
C--- Save blade definition with chord,twist and body velocity
      DO I=1, II
        BETA0DEG = BETA0(I)*180.0/PI
        WRITE(LU,1200) XI(I),CH(I),BETA0DEG,UBODY(I)
      END DO
C--- Duct velocity
      WRITE(LU,1113)
      WRITE(LU,1200) URDUCT
C--- Save added velocity components
      IF(NADD.GT.1) THEN
       WRITE(LU,1114) NADD
       DO I=1, NADD
         WRITE(LU,1200) RADD(I), UADD(I), VADD(I)
       END DO
       WRITE(*,*) 'External slipstream included in save file'
      ENDIF
C
      CLOSE(LU)
      RETURN
C
 90   WRITE(*,*) 'Bad filename.'
      WRITE(*,*) 'Current rotor not saved.'
      RETURN
C 
C...................................................................
 1000 FORMAT(A)
 1050 FORMAT(/' *** Converged operating solution does not exist ***')
 1100 FORMAT('XROTOR VERSION: ',F5.2/A)
 1102 FORMAT('!         Rho          Vso          Rmu           Alt')
 1103 FORMAT('!         Rad          Vel          Adv          Rake')
 1104 FORMAT('!         XI0          XIW')
 1105 FORMAT('!  Naero'/1(1X,I5))
 1106 FORMAT('!   Xisection')
 1107 FORMAT('!       A0deg        dCLdA        CLmax         CLmin')
 1108 FORMAT('!  dCLdAstall     dCLstall      Cmconst         Mcrit')
 1109 FORMAT('!       CDmin      CLCDmin     dCDdCL^2')
 1110 FORMAT('!       REref        REexp')
 1111 FORMAT('!LVDuct  LDuct   LWind'/3(1X,L2,5X))
 1112 FORMAT('!   II Nblds'/2(1X,I5),
     &      /'!         r/R          C/R     Beta0deg         Ubody')
 1113 FORMAT('!      URDuct')
 1114 FORMAT('!Nadd'/1(1X,I5),
     &      /'!        Radd         Uadd         Vadd')
 1200 FORMAT(5(1X,G12.5))
C
Cx123456789012x123456789012x123456789012x123456789012x123456789012
C!         Rho          Vso          Rmu           Alt')
C
      END ! SAVE



      SUBROUTINE LOADOLD(FNAME1)
C---------------------------------------------------------
C     Reads in previously saved rotor in old XROTOR format
C---------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
      CHARACTER*128 LINE
      GREEK = .FALSE.
C
      LU = LUTEMP
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter filename^',FNAME)
C
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=200)
      READ(LU,1000,ERR=210) NAME
      READ(LU,*,ERR=210) IIX,NBLDS
      READ(LU,*,ERR=210) RHO,VSO,RMU
      READ(LU,*,ERR=210) RAD,VEL,ADV
      READ(LU,*,ERR=210) XI0,XW0
C--- Aero data
      READ(LU,*,ERR=210) A0,DCLDA,CLMAX,CLMIN
      READ(LU,*,ERR=210) CDMIN,CLDMIN,DCDCL2
      READ(LU,*,ERR=210) REREF,REXP
C
      READ(LU,*,ERR=210) FREE, DUCT, WIND
      WRITE(*,*)
      IF(     FREE) WRITE(*,*) 'Self-deforming wake option set'
      IF(.NOT.FREE) WRITE(*,*) 'Rigid wake option set'  
      IF(     DUCT) WRITE(*,*) 'Duct option set'
      IF(.NOT.DUCT) WRITE(*,*) 'Free-tip option set'
      IF(     WIND) WRITE(*,*) 'Windmill plotting mode set'
      IF(.NOT.WIND) WRITE(*,*) 'Propeller plotting mode set'
      WRITE(*,*) 'Reading prop definition...'
      DO I=1, IIX
        READ(LU,*,ERR=210) XI(I),CH(I),BETA(I),UBODY(I)
        BETA0(I) = BETA(I)
      END DO
      WRITE(*,*) ' '
C
      URDUCT = 1.0
      READ(LU,*,END=19) URDUCT
C
C---- try to read slipstream data
 19   NADD = 0
      DO I=1, IX
        READ(LU,*,END=21) RADD(I), UADD(I), VADD(I)
      END DO
      I = IX+1
 21   CONTINUE
      IF(I.GT.2) THEN
       NADD = I-1
       WRITE(*,*) 'Slipstream profiles read in'
      ENDIF
      CLOSE(LU)
C
      ALT = 999.0
      CONV = .FALSE.
C
C--- Check for number of analysis stations to use
      IF(IIX.NE.II) THEN
 22     WRITE(*,23) IIX,II,II
        READ(*,24) LINE
        IF(LINE.NE.' ') THEN
          READ(LINE,*,ERR=22) II
        ENDIF
 23     FORMAT(/'Read  # input stations = ',I3,
     &         /'Using # blade stations = ',I3,
     &         /'Enter # stations or <cr> for ',I3,' ',$)
 24     FORMAT(A)
      ENDIF
C
C---- spline blade geometry to "old" radial locations
      DO I = 1, IIX
        W1(I) = XI(I)
        W2(I) = CH(I)
        W4(I) = BETA(I)
        W6(I) = UBODY(I)
      ENDDO
      CALL SPLINE(W2,W3,W1,IIX)
      CALL SPLINE(W4,W5,W1,IIX)
      CALL SPLINE(W6,W7,W1,IIX)
C
C---- set radial stations for built-in distribution scheme
      CALL SETX
      CALL XWINIT
C
C---- interpolate read-in geometry to generated radial stations
      DO I = 1, II
        CH(I)    = SEVAL(XI(I),W2,W3,W1,IIX)
        BETA(I)  = SEVAL(XI(I),W4,W5,W1,IIX)
        UBODY(I) = SEVAL(XI(I),W6,W7,W1,IIX)
        BETA0(I) = BETA(I)
      ENDDO
      IINF = II + II/2
C
C--- Install defined aero properties for section(s), 
C    supply defaults for those not stored in XROTOR dataset
      DCL_STALL   =  0.1 ! CL increment from incipient to total stall
      DCLDA_STALL =  0.1 ! stalled lift curve slope /radian
      CMCON       = -0.1 ! Cm  (for pitch-axis moments)
      MCRIT       =  0.8 ! critical Mach #
      NAERO = 1
      XISECT = 0.0
      CALL PUTAERO(NAERO,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      CALL SETIAERO
C
C---- calculate current operating point
      CALL APER(4,2,.TRUE.)
      IF(CONV) CALL OUTPUT(LUWRIT)
C
C---- define design quantities for design of MIL prop with same parameters
      RADDES = RAD
      VELDES = VEL
      ADVDES = 0.
      RPMDES = VEL/(RAD*ADV) * 30.0/PI
      R0DES = XI0*RAD
      RWDES = XW0*RAD
      TDDES = TTOT*RHO*VEL**2*RAD**2
      PDDES = PTOT*RHO*VEL**3*RAD**2
      DEST = .FALSE.
      DESP = .TRUE.
      DO I=1, II
        CLDES(I) = CL(I)
      ENDDO
      CLDES0 = 0.
C
C---- rotor now exists
      LROTOR = .TRUE.
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
 1000 FORMAT(A)
 1010 FORMAT(' File  ',A,' not found'/)
 1020 FORMAT(' File  ',A,' has incompatible format'/
     &       ' Loading not completed'/)
      END ! LOADOLD


      SUBROUTINE LOAD(FNAME1)
C------------------------------------------------------------------------
C     Reads in previously saved rotor in new XROTOR_Version >= 6.9 format
C     This format saves more information and can have optional comment 
C     lines beginning with a ! character.
C------------------------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      CHARACTER*(*) FNAME1
      CHARACTER*128 LINE
      GREEK = .FALSE.
C
      LU = LUTEMP
C
      FNAME = FNAME1
      IF(FNAME(1:1) .EQ. ' ') CALL ASKS('Enter filename^',FNAME)
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=200)
C
C--- Check for new format/old format XROTOR file
      CALL RDLINE(LU,LINE)
      IF(LINE.EQ.'END' .OR. LINE.EQ.'ERR') GO TO 210
      IF(LINE(1:6).NE.'XROTOR') THEN
        WRITE(*,*) 'Old XROTOR file found, reading old format...'
        CLOSE(LU)
        CALL LOADOLD(FNAME)
        RETURN
      ENDIF
      READ(LINE(17:22),*) FILEVERS
      WRITE(*,1005) FILEVERS
C
C
C--- Case title
      CALL RDLINE(LU,LINE)
      NAME = LINE
C
      CALL RDLINE(LU,LINE)
      READ(LINE,*,ERR=210) RHO,VSO,RMU,ALT
      CALL RDLINE(LU,LINE)
      READ(LINE,*,ERR=210) RAD,VEL,ADV,RAKE
      CALL RDLINE(LU,LINE)
      READ(LINE,*,ERR=210) XI0,XW0
      CALL RDLINE(LU,LINE)
C
C--- Read aero section definitions
      READ(LINE,*,ERR=210) NAERO
      DO N = 1, NAERO
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) XISECT
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) A0DEG,DCLDA,CLMAX,CLMIN
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) DCLDA_STALL,DCL_STALL,CMCON,MCRIT
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) CDMIN,CLDMIN,DCDCL2
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) REREF,REXP
C
        A0 = A0DEG *PI/180.0
        CALL PUTAERO(N,XISECT,A0,CLMAX,CLMIN,
     &               DCLDA,DCLDA_STALL,DCL_STALL,
     &               CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      END DO
C
C--- Read flags for wake, duct and windmill modes
      CALL RDLINE(LU,LINE)
      READ(LINE,*,ERR=210) FREE, DUCT, WIND
C
      WRITE(*,*)
      IF(     FREE) WRITE(*,*) 'Self-deforming wake option set'
      IF(.NOT.FREE) WRITE(*,*) 'Rigid wake option set'  
      IF(     DUCT) WRITE(*,*) 'Duct option set'
      IF(.NOT.DUCT) WRITE(*,*) 'Free-tip option set'
      IF(     WIND) WRITE(*,*) 'Windmill plotting mode set'
      IF(.NOT.WIND) WRITE(*,*) 'Propeller plotting mode set'
C
      WRITE(*,*) ' '
      CALL RDLINE(LU,LINE)
      IF(LINE.EQ.'END' .OR. LINE.EQ.'ERR') GO TO 210       
      READ(LINE,*,ERR=210) IIX,NBLDS
      DO I=1, IIX
        CALL RDLINE(LU,LINE)
        READ(LINE,*,ERR=210) XI(I),CH(I),BETADEG,UBODY(I)
        BETA(I) = BETADEG *PI/180.0
        BETA0(I) = BETA(I)
cc        write(*,*) 'load i,ch,beta ',i,ch(i),beta(i)
      END DO
C
C--- Optional duct velocity
      URDUCT = 1.0
      CALL RDLINE(LU,LINE)
      IF(LINE.EQ.'END' .OR. LINE.EQ.'ERR') GO TO 19       
      READ(LINE,*,END=19) URDUCT
C
C---- Optional slipstream velocities
 19   NADD = 0
      CALL RDLINE(LU,LINE)
      IF(LINE.EQ.'END' .OR. LINE.EQ.'ERR') GO TO 21       
      READ(LINE,*,END=21) NADD
      IF(NADD.GT.IX) THEN
        NADD = IX
        WRITE(*,*) 'Warning, slipstream data terminated at ',IX
      ENDIF
      DO I=1, NADD
        CALL RDLINE(LU,LINE)
        IF(LINE.EQ.'END' .OR. LINE.EQ.'ERR') GO TO 20       
        READ(LINE,*,ERR=20,END=20) RADD(I), UADD(I), VADD(I)
      END DO
      IF(I.LT.NADD) THEN
        NADD = I-1
        WRITE(*,*) 'Warning, slipstream data terminated at ',NADD
      ENDIF
      GO TO 21
C
 20   IF(I.GT.2) THEN
       NADD = I-1
      ENDIF
C
 21   CLOSE(LU)
      IF(NADD.GT.1) THEN
       WRITE(*,*)
       WRITE(*,*) 'Slipstream profiles read with #points ',NADD
      ENDIF
C
      CONV = .FALSE.
C
C--- Check for number of analysis stations to use
      IF(IIX.NE.II) THEN
 22     WRITE(*,23) IIX,II,II
        READ(*,24) LINE
        IF(LINE.NE.' ') THEN
          READ(LINE,*,ERR=22) II
        ENDIF
 23     FORMAT(/'Read  # input stations = ',I3,
     &         /'Using # blade stations = ',I3,
     &         /'Enter # stations or <cr> for ',I3,' ',$)
 24     FORMAT(A)
      ENDIF
C
C---- spline blade geometry to "old" radial locations
      DO I = 1, IIX
        W1(I) = XI(I)
        W2(I) = CH(I)
        W4(I) = BETA(I)
        W6(I) = UBODY(I)
      ENDDO
      CALL SPLINE(W2,W3,W1,IIX)
      CALL SPLINE(W4,W5,W1,IIX)
      CALL SPLINE(W6,W7,W1,IIX)
C
C---- set radial stations for built-in distribution scheme
      CALL SETX
      CALL XWINIT
C
C---- interpolate read-in geometry to generated radial stations
      DO I = 1, II
        CH(I)    = SEVAL(XI(I),W2,W3,W1,IIX)
        BETA(I)  = SEVAL(XI(I),W4,W5,W1,IIX)
        UBODY(I) = SEVAL(XI(I),W6,W7,W1,IIX)
        BETA0(I) = BETA(I)
cc        write(*,*) 'load trp i,ch,beta ',i,ch(i),beta(i)
      ENDDO
      IINF = II + II/2
C
      CALL SETIAERO
C---- calculate current operating point
      CALL APER(4,2,.TRUE.)
      IF(CONV) CALL OUTPUT(LUWRIT)
C
C---- define design quantities for design of MIL prop with same parameters
      RADDES = RAD
      VELDES = VEL
      ADVDES = 0.
      RPMDES = VEL/(RAD*ADV) * 30.0/PI
      R0DES = XI0*RAD
      RWDES = XW0*RAD
      TDDES = TTOT*RHO*VEL**2*RAD**2
      PDDES = PTOT*RHO*VEL**3*RAD**2
      DEST = .FALSE.
      DESP = .TRUE.
      DO I=1, II
        CLDES(I) = CL(I)
      ENDDO
      CLDES0 = 0.
C
C---- rotor now exists
      LROTOR = .TRUE.
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
 1000 FORMAT(A)
 1005 FORMAT(' Reading file from XROTOR Version ',F5.2)
 1010 FORMAT(' File  ',A,' not found'/)
 1020 FORMAT(' File  ',A,' has incompatible format'/
     &       ' Loading not completed'/)
      END ! LOAD



      SUBROUTINE RDLINE(LUN,LINE)
C...Purpose  Read a non-comment line from the input file 
C...Input    Data read from unit LUN
C...Output   LINE  Character string with input line
C                  LINE is set to 'END' for end or errors
C
      CHARACTER*(*) LINE
C
 1000 FORMAT(A)
   20 READ (LUN,1000,END=80,ERR=90) LINE
C
C---- skip comment line
      IF(INDEX('!#',LINE(1:1)) .NE. 0) GO TO 20
C
C---- skip blank line
      IF(LINE.EQ.' ') GO TO 20
C
C---- normal return after significant line
      RETURN
C
   80 LINE = 'END '
      RETURN
C
   90 LINE = 'ERR '
      RETURN
      END





