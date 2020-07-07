C***********************************************************************
C    Module:  xaero.f
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


C--- Aero data stored for one or more radial aerodynamic sections
C   
C-- aero data quantities for each defined radial aerodynamic section
C   NAERO       Number of aerodynamic datasets defined (NAERO>=1)
C   XIAERO      Radial station r/R where aero dataset is defined
C   AERODATA    Aerodynamic definition of the blade section at XIAERO
C               AERODATA( 1,x) = A0 (angle of zero lift)
C               AERODATA( 2,x) = CLMAX (Max CL)
C               AERODATA( 3,x) = CLMIN (Min CL)
C               AERODATA( 4,x) = DCLDA (Incompressible 2-D lift curve slope)
C               AERODATA( 5,x) = DCLDA_STALL (2-D lift curve slope at stall)
C               AERODATA( 6,x) = DCL_STALL (CL increment, onset to full stall)
C               AERODATA( 7,x) = CDMIN (Minimum drag coefficient value)
C               AERODATA( 8,x) = CLDMIN (Lift at minimum drag value)
C               AERODATA( 9,x) = DCDCL2 (Parabolic drag param d(Cd)/dCL^2)
C               AERODATA(10,x) = CMCON (Incompressible 2-D pitching moment)
C               AERODATA(11,x) = REREF (reference Reynold's number)
C               AERODATA(12,x) = REXP (Reynold's number exponent Cd~Re^REXP)
C               AERODATA(13,x) = MCRIT (critical Mach #)
C               AERODATA(14,x) = TOC (thickness/chord)



      SUBROUTINE AERO
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 COMAND
      CHARACTER*132 COMARG,LINE
      LOGICAL ERROR, LANS
      PARAMETER (NMACH = 10)
      DIMENSION AMACH(NMACH)
C
C--------------------------------------------------------------
C     Display, change, or write out airfoil section properties
C--------------------------------------------------------------
C
 1    FORMAT(A)
      GREEK = .FALSE.
      NM = 0
C      GO TO 10
C
 900  WRITE(*,920)
      DO N = 1, NAERO
         CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &                DCLDA,DCLDA_STALL,DCL_STALL,
     &                CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
        WRITE(*,930) N,XIAERO(N),CLMAX,CLMIN,CDMIN,MCRIT,REXP,REREF
      END DO
C
 920  FORMAT(/'Defined aerodynamic sections: ',
     &       /'  N      r/R   CLmax   CLmin    CDmin',
     &        '   Mcrit   REexp        REref')
 930  FORMAT(I3,3(1X,F7.4),1X,F8.5,1X,F7.4,1X,F7.4,1X,G12.4)
C   N      r/R   CLmax   CLmin    CDmin   Mcrit   REexp        REref
Cxiiixffffffffxfffffffxfffffffxffffffffxfffffffxfffffffxffffffffffff
C
C
      CALL ASKC('.AERO^',COMAND,COMARG)
C
      IF(COMAND.EQ.'    ') THEN
       IF(.NOT.CONV) CALL SETIAERO
       RETURN
      ENDIF
C
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'DISP') GO TO 10
      IF(COMAND.EQ.'NEW')  GO TO 12
      IF(COMAND.EQ.'DEL')  GO TO 13
      IF(COMAND.EQ.'EDIT') GO TO 15
      IF(COMAND.EQ.'PLOT') GO TO 60
      IF(COMAND.EQ.'READ') GO TO 20
      IF(COMAND.EQ.'WRIT') GO TO 24
      IF(COMAND.EQ.'ANNO') GO TO 65
      IF(COMAND.EQ.'HARD') GO TO 68
      WRITE(*,1000) COMAND
      GO TO 900
C
 1000 FORMAT(1X,A4,' command not recognized.' //
     &             '  Type "?" for list, <Return> to exit menu.')
 1100 FORMAT(/'   DISP  Display airfoil characteristics'
     &       /'   NEW   Create a new aero section'
     &       /'   DEL   Delete an aero section'
     &       /'   EDIT  Edit section aero data'
     &       /'   READ  Read airfoil characteristics from disk file'
     &       /'   WRIT  Write airfoil characteristics to disk file'
     &       /'   PLOT  Plot airfoil characteristics'
     &       /'   ANNO  Annotate current plot'
     &       /'   HARD  Hardcopy current plot')
C
C
C-------------------------------------------------------------
C--- Display aero data
 10   DO N = 1, NAERO
       CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &              DCLDA,DCLDA_STALL,DCL_STALL,
     &              CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
       WRITE(*,1420) N,XISECT
       A0DEG = A0*180./PI
       WRITE(*,1500) A0DEG,CDMIN
       WRITE(*,1510) DCLDA,CLDMIN
       WRITE(*,1515) DCLDA_STALL,DCDCL2
       WRITE(*,1520) CLMAX,REREF
       WRITE(*,1530) CLMIN,REXP
       WRITE(*,1540) DCL_STALL,CMCON
       WRITE(*,1550) MCRIT
      END DO
      GO TO 900
C
C-------------------------------------------------------------
C--- Define new aero section
 12   IF(NAERO.GT.1) THEN
        WRITE(*,1200)
        READ(*,1) LINE
        IF(LINE.EQ.' ') GO TO 900
        READ(LINE,*,ERR=12) N
        IF(N.LT.0 .OR. N.GT.NAERO) GO TO 900
       ELSE
        N = 1
      ENDIF
      CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
 121  WRITE(*,1220) N,XISECT
      READ(*,1) LINE
      READ(LINE,*,ERR=121) XISECT
      IF(XISECT.LT.0.0 .OR. XISECT.GT.1.0) THEN
        WRITE(*,*) '*** Section r/R must be in range 0->1'
        GO TO 121
      ENDIF
      CALL PUTAERO(NAERO+1,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      NAERO = NAERO+1
C--- Sort aero data sections by XI location
      CALL SORTAR(NAERO,XIAERO,AERODATA(1,1),NDX)
      CONV = .FALSE.
      GO TO 900
C
 1200 FORMAT(/'Enter index of aero section to copy: ')
 1220 FORMAT(/'Section # ',I3,' @ r/R = ',F10.4,
     &       /'Enter r/R for new section: ')
C
C-------------------------------------------------------------
C--- Delete aero section
 13   IF(NAERO.GT.1) THEN
        WRITE(*,1300)
        READ(*,1) LINE
        IF(LINE.EQ.' ') GO TO 900
        READ(LINE,*,ERR=13) NDEL
        IF(NDEL.LT.0 .OR. NDEL.GT.NAERO) GO TO 900
       ELSE
        WRITE(*,*) '*** Cannot delete all aero sections'
        GO TO 900
      ENDIF
      WRITE(*,1320) NDEL,XIAERO(NDEL)
      CALL ASKL('Confirm delete?^',LANS)
      IF(.NOT.LANS) GO TO 900
      DO N = NDEL+1, NAERO
        CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &               DCLDA,DCLDA_STALL,DCL_STALL,
     &               CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
        CALL PUTAERO(N-1,XISECT,A0,CLMAX,CLMIN,
     &               DCLDA,DCLDA_STALL,DCL_STALL,
     &               CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      END DO
      NAERO = NAERO-1
      CONV = .FALSE.
      GO TO 900
C
 1300 FORMAT(/'Enter index of aero section to delete: ')
 1320 FORMAT(/'Section # ',I3,' @ r/R = ',F10.4)
C
C-------------------------------------------------------------
C--- Edit aero data
C-------------------------------------------------------------
 15   IF(NAERO.GT.1) THEN
        WRITE(*,1400)
        READ(*,1) LINE
        IF(LINE.EQ.' ') GO TO 900
        READ(LINE,*,ERR=15) N
        IF(N.LT.0 .OR. N.GT.NAERO) GO TO 900
       ELSE
        N = 1
      ENDIF
      CALL AEROEDIT(N)
      GO TO 900
C
C-------------------------------------------------------------
C--- Read aero data from file
 20   CALL ASKS('Enter aero data filename^',FNAME)
      IF(FNAME.EQ.' ') GO TO 900
      OPEN(LUSAVE,FILE=FNAME,STATUS='OLD',ERR=900)
      DO N = 1, NAX
C
c       READ(LUSAVE,1420,END=26,ERR=26) NN,XISECT
c       READ(LUSAVE,1500) A0DEG,CDMIN
c       READ(LUSAVE,1510) DCLDA,CLDMIN
c       READ(LUSAVE,1515) DCLDA_STALL,DCDCL2
c       READ(LUSAVE,1520) CLMAX,REREF
c       READ(LUSAVE,1530) CLMIN,REXP
c       READ(LUSAVE,1540) DCL_STALL,CMCON
c       READ(LUSAVE,1540) MCRIT
C
       READ(LUSAVE,1,END=26,ERR=26) LINE
       READ(LUSAVE,1,END=26,ERR=26) LINE
       READ(LINE(9:12),*) NN
       READ(LINE(20:30),*) XISECT
       READ(LUSAVE,1) LINE
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) A0DEG
       READ(LINE(62:77),*) CDMIN
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) DCLDA
       READ(LINE(62:77),*) CLDMIN
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) DCLDA_STALL
       READ(LINE(62:77),*) DCDCL2
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) CLMAX
       READ(LINE(62:77),*) REREF
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) CLMIN
       READ(LINE(62:77),*) REXP
       READ(LUSAVE,1) LINE
       READ(LINE(24:39),*) DCL_STALL
       READ(LINE(62:77),*) CMCON
       READ(LUSAVE,1) LINE
       READ(LINE(62:77),*) MCRIT
       READ(LUSAVE,1,END=26,ERR=26) LINE
C
       A0 = A0DEG*180./PI
       CALL PUTAERO(N,XISECT,A0,CLMAX,CLMIN,
     &              DCLDA,DCLDA_STALL,DCL_STALL,
     &              CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      END DO
 26   CLOSE(LUSAVE)
      NAERO = N-1
      CALL SORTAR(NAERO,XIAERO,AERODATA(1,1),NDX)
      CONV = .FALSE.
      GO TO 900
C
C-------------------------------------------------------------
C--- Write aero data to file
 24   CALL OPFILE(LUSAVE,SAVFIL)
      DO N = 1, NAERO
       CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &              DCLDA,DCLDA_STALL,DCL_STALL,
     &              CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
       A0DEG = A0*180./PI
       WRITE(LUSAVE,1420) N,XISECT
       WRITE(LUSAVE,1500) A0DEG,CDMIN
       WRITE(LUSAVE,1510) DCLDA,CLDMIN
       WRITE(LUSAVE,1515) DCLDA_STALL,DCDCL2
       WRITE(LUSAVE,1520) CLMAX,REREF
       WRITE(LUSAVE,1530) CLMIN,REXP
       WRITE(LUSAVE,1540) DCL_STALL,CMCON
       WRITE(LUSAVE,1550) MCRIT
      END DO
      CLOSE(LUSAVE)
      GO TO 900
C
C-------------------------------------------------------------
C---- plot the aero data for a defined section
C
 60   IF(NAERO.GT.1) THEN
        WRITE(*,1400)
        READ(*,1) LINE
        IF(LINE.EQ.' ') GO TO 900
        READ(LINE,*,ERR=60) N
        IF(N.LT.0 .OR. N.GT.NAERO) GO TO 900
       ELSE
        N = 1
      ENDIF
      CALL GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C
C--- Get Mach #'s to plot
      WRITE(*,2060) 
      READ(*,1) LINE
      IF(LINE.EQ.' ' .AND. NM.LE.0) LINE = 'S'
      IF(LINE.NE.' ') THEN
        NM = NMACH
        CALL GETFLT(LINE,AMACH,NM,ERROR)          
C--- No intelligible Mach #'s entered, use standard set
        IF(ERROR) THEN
         AMACH(1) = 0.0
         AMACH(2) = 0.4
         AMACH(3) = 0.5
         AMACH(4) = 0.6
         AMACH(5) = 0.7
         AMACH(6) = 0.8
         AMACH(7) = 0.9
         NM = 7
        ENDIF
      ENDIF
C
      CALL AEROPLT(N,XISECT,NM,AMACH,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      GO TO 900
C
 2060 FORMAT(/' Enter Mach #s to plot or <cr> for standard set')
C
C-------------------------------------------------------------
 65   IF(LPLOT) CALL ANNOT(1.2*CSIZE)
      GO TO 900
C
C-------------------------------------------------------------
 68   CALL PLEND
      CALL REPLOT(IDEVRP)
      GO TO 900
C
C.....................................................................
C
 1400 FORMAT(/'Enter index of aero section to process: ')
 1420 FORMAT(/' Sect# = ',I3,' r/R = ',F10.4)
C
 1500 FORMAT( 1X,72('=')/
     &       ' Zero-lift alpha (deg):',F7.2,8X,
     &        'Minimum Cd           :',F7.4)
 1510 FORMAT(' d(Cl)/d(alpha)       :',F7.3,8X,
     &        'Cl at minimum Cd     :',F6.3)
 1515 FORMAT(' d(Cl)/d(alpha)@stall :',F7.3,8X,
     &        'd(Cd)/d(Cl**2)       :',F7.4)
 1520 FORMAT(' Maximum Cl           :',F6.2,9X,
     &        'Reference Re number  :',F9.0)
 1530 FORMAT(' Minimum Cl           :',F6.2,9X,
     &        'Re scaling exponent  :',F8.4)
 1540 FORMAT(' Cl increment to stall:',F7.3,8X,
     &        'Cm                   :',F7.3)
 1550 FORMAT('                       ',15X,
     &        'Mcrit                :',F7.3/
     &         1X,72('='))
C
      END ! AERO



      SUBROUTINE AEROEDIT(NSEC)
C--------------------------------------------------------------
C     Edit airfoil section properties
C--------------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      CHARACTER*4 COMAND
      CHARACTER*132 COMARG,LINE
      LOGICAL ERROR, LANS, LNUMCMD
      PARAMETER (NMACH = 10)
      DIMENSION AMACH(NMACH)
C
 1    FORMAT(A)
      GREEK = .FALSE.
C
      IF(NSEC.LT.0 .OR. NSEC.GT.NAERO) THEN
        WRITE(*,*) 'AEROEDIT: section index out of bounds: ',NSEC
        RETURN
      ENDIF
C
      CALL GETAERO(NSEC,XISECT,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      GO TO 10
C
C
 900  CALL ASKC('.EDIT^',COMAND,COMARG)
C
      IF(COMAND.EQ.'    ') THEN
        CALL PUTAERO(NSEC,XISECT,A0,CLMAX,CLMIN,
     &               DCLDA,DCLDA_STALL,DCL_STALL,
     &               CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C--- Sort aero data sections by XI location
       CALL SORTAR(NAERO,XIAERO,AERODATA(1,1),NDX)
       IF(.NOT.CONV) CALL SETIAERO
       RETURN
      ENDIF
C
      IF(COMAND.EQ.'?   ') WRITE(*,1100)
      IF(COMAND.EQ.'?   ') GO TO 900
      IF(COMAND.EQ.'DISP') GO TO 10
      IF(COMAND.EQ.'LIFT') GO TO 30
      IF(COMAND.EQ.'DRAG') GO TO 40
      IF(COMAND.EQ.'MOVE') GO TO 20
      IF(COMAND.EQ.'REFL') GO TO 50
      IF(COMAND.EQ.'PLOT') GO TO 60
C
      LNUMCMD = .TRUE.
      IF(COMAND.EQ.'1')    GO TO 31
      IF(COMAND.EQ.'2')    GO TO 32
      IF(COMAND.EQ.'3')    GO TO 33
      IF(COMAND.EQ.'4')    GO TO 34
      IF(COMAND.EQ.'5')    GO TO 35
      IF(COMAND.EQ.'6')    GO TO 36
      IF(COMAND.EQ.'7')    GO TO 41
      IF(COMAND.EQ.'8')    GO TO 42
      IF(COMAND.EQ.'9')    GO TO 43
      IF(COMAND.EQ.'10')   GO TO 44
      IF(COMAND.EQ.'11')   GO TO 45
      IF(COMAND.EQ.'12')   GO TO 37
      IF(COMAND.EQ.'13')   GO TO 46
C
      WRITE(*,1000) COMAND
      GO TO 900
C
 1000 FORMAT(1X,A4,' command not recognized.' //
     &             '  Type "?" for list, <Return> to exit menu.')
 1100 FORMAT(/'   DISP  Display section aero  characteristics'
     &       /'   LIFT  Change section lift characteristics'
     &       /'   DRAG  Change section drag characteristics'
     &       /'   MOVE  Change section r/R location'
     &       /'   REFL  Reflect CL and CD curves (for windmill)'
     &       /'   PLOT  Plot airfoil characteristics')
C
C
C-------------------------------------------------------------
C--- Display aero data
 10   WRITE(*,1420) NSEC,XISECT
      A0DEG = A0*180./PI
      WRITE(*,1500) A0DEG,CDMIN
      WRITE(*,1510) DCLDA,CLDMIN
      WRITE(*,1515) DCLDA_STALL,DCDCL2
      WRITE(*,1520) CLMAX,REREF
      WRITE(*,1530) CLMIN,REXP
      WRITE(*,1540) DCL_STALL,CMCON
      WRITE(*,1550) MCRIT
      GO TO 900
C
C-------------------------------------------------------------
C--- Change section r/R location
 20   IF(NSEC.EQ.1) THEN
        WRITE(*,*) '*** Cannot move section #1'
        GO TO 900
      ENDIF
C
 21   WRITE(*,1220) NSEC,XISECT
      READ(*,1) LINE
      IF(LINE.EQ.' ') GO TO 900
      READ(LINE,*,ERR=21) XISECT
      IF(XISECT.LT.0.0 .OR. XISECT.GT.1.0) THEN
        WRITE(*,*) '*** Section r/R must be in range 0->1'
        GO TO 21
      ENDIF
      CONV = .FALSE.
      GO TO 10
C
 1220 FORMAT(/'Section # ',I3,' @ r/R = ',F10.4,
     &       /'Enter new r/R for section: ')
C
C-------------------------------------------------------------
C--- Edit lift aero data
C-------------------------------------------------------------
 30   LNUMCMD = .FALSE.
C
 31   A0DEG = A0*180./PI
      WRITE(*,2031) A0DEG
 2031 FORMAT(/' Zero-lift alpha (deg)  :',F9.4)
      CALL READR(1,A0DEG,ERROR)
      IF(ERROR) GO TO 31
      A0 = A0DEG * PI/180.
      IF(LNUMCMD) GO TO 39
C
 32   WRITE(*,2032) DCLDA
 2032 FORMAT(/' d(Cl)/d(alpha)  (/rad) :',F9.5)
      CALL READR(1,DCLDA,ERROR)
      IF(ERROR) GO TO 32
      IF(LNUMCMD) GO TO 39

 33   WRITE(*,2033) DCLDA_STALL
 2033 FORMAT(/' d(Cl)/d(alpha) stall  (/rad) :',F9.5)
      CALL READR(1,DCLDA_STALL,ERROR)
      IF(ERROR) GO TO 33
      IF(LNUMCMD) GO TO 39
C
 34   WRITE(*,2034) CLMAX
 2034 FORMAT(/' Maximum Cl             :',F9.4)
      CALL READR(1,CLMAX,ERROR)
      IF(ERROR) GO TO 34
      IF(LNUMCMD) GO TO 39
C
 35   WRITE(*,2035) CLMIN
 2035 FORMAT(/' Minimum Cl             :',F9.4)
      CALL READR(1,CLMIN,ERROR)
      IF(ERROR) GO TO 34
      IF(LNUMCMD) GO TO 39
C
 36   WRITE(*,2036) DCL_STALL
 2036 FORMAT(/' Cl increment to stall  :',F9.5)
      CALL READR(1,DCL_STALL,ERROR)
      IF(ERROR) GO TO 36
      IF(LNUMCMD) GO TO 39
C
 37   WRITE(*,2037) CMCON
 2037 FORMAT(/' Cm                     :',F9.5)
      CALL READR(1,CMCON,ERROR)
      IF(ERROR) GO TO 37
C
 39   CONV = .FALSE.
      GO TO 10
C
C-------------------------------------------------------------
C--- Edit drag aero data
C-------------------------------------------------------------
 40   LNUMCMD = .FALSE.
C
 41   WRITE(*,2041) CDMIN
 2041 FORMAT(/' Minimum Cd             :',F10.6)
      CALL READR(1,CDMIN,ERROR)
      IF(ERROR) GO TO 41
      IF(LNUMCMD) GO TO 48
C
   42 WRITE(*,2042) CLDMIN
 2042 FORMAT(/' Cl at minimum Cd       :',F10.5)
      CALL READR(1,CLDMIN,ERROR)
      IF(ERROR) GO TO 42
      IF(LNUMCMD) GO TO 48
C
   43 WRITE(*,2043) DCDCL2
 2043 FORMAT(/' d(Cd)/d(Cl**2)         :',F10.6)
      CALL READR(1,DCDCL2,ERROR)
      IF(ERROR) GO TO 43
      IF(LNUMCMD) GO TO 48
C
   44 WRITE(*,2044) REREF
 2044 FORMAT(/' Reference Re number    :',F12.0)
      CALL READR(1,REREF,ERROR)
      IF(ERROR) GO TO 44
      IF(LNUMCMD) GO TO 48
C
   45 WRITE(*,2045) REXP
 2045 FORMAT(/' Re scaling exponent    :',F10.4)
      CALL READR(1,REXP,ERROR)
      IF(ERROR) GO TO 45
      IF(LNUMCMD) GO TO 48
C
 46   WRITE(*,2046) MCRIT
 2046 FORMAT(/' Mcrit                  :',F9.5)
      CALL READR(1,MCRIT,ERROR)
      IF(ERROR) GO TO 46
C
 48   CONV = .FALSE.
      GO TO 10
C
C-------------------------------------------------------------
C--- Reflect CL-alpha curve for "upside-down" windmill airfoils
C--- Sign reversals apply to CL and alpha
 50   A0 = -A0
      CLDMIN = -CLDMIN
      TMP = CLMIN
      CLMIN = -CLMAX
      CLMAX = -TMP
      CMCON = -CMCON
      CONV = .FALSE.
C
C---- switch windmill flag for use in plotting
      WIND = .NOT. WIND
      WRITE(*,*)
      IF(     WIND) WRITE(*,*) 'Windmill-mode plotting flag set'
      IF(.NOT.WIND) WRITE(*,*) 'Propeller-mode plotting flag set'
      GO TO 10
C
C-------------------------------------------------------------
C---- plot the aero data for section
C--- Get Mach #'s to plot
 60   WRITE(*,2060) 
      READ(*,1) LINE
      IF(LINE.EQ.' ' .AND. NM.LE.0) LINE = 'S'
      IF(LINE.NE.' ') THEN
        NM = NMACH
        CALL GETFLT(LINE,AMACH,NM,ERROR)          
C--- No intelligible Mach #'s entered, use standard set
        IF(ERROR) THEN
         AMACH(1) = 0.0
         AMACH(2) = 0.4
         AMACH(3) = 0.5
         AMACH(4) = 0.6
         AMACH(5) = 0.7
         AMACH(6) = 0.8
         AMACH(7) = 0.9
         NM = 7
        ENDIF
      ENDIF
      CALL AEROPLT(NSEC,XISECT,NM,AMACH,A0,CLMAX,CLMIN,
     &             DCLDA,DCLDA_STALL,DCL_STALL,
     &             CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
      GO TO 900
C
C.....................................................................
C
 1400 FORMAT(/'Enter index of aero section to process: ')
 1420 FORMAT(/' Sect# = ',I3,' r/R = ',F10.4)
C
 1500 FORMAT( 1X,72('=')/
     &       ' 1) Zero-lift alpha (deg):',F7.2,6X,
     &       ' 7) Minimum Cd           :',F7.4)
 1510 FORMAT(' 2) d(Cl)/d(alpha)       :',F7.3,6X,
     &       ' 8) Cl at minimum Cd     :',F6.3)
 1515 FORMAT(' 3) d(Cl)/d(alpha)@stall :',F7.3,6X,
     &       ' 9) d(Cd)/d(Cl**2)       :',F7.4)
 1520 FORMAT(' 4) Maximum Cl           :',F6.2,7X,
     &       '10) Reference Re number  :',F9.0)
 1530 FORMAT(' 5) Minimum Cl           :',F6.2,7X,
     &       '11) Re scaling exponent  :',F8.4)
 1540 FORMAT(' 6) Cl increment to stall:',F7.3,6X,
     &       '12) Cm                   :',F7.3)
 1550 FORMAT('                          ',13X,
     &       '13) Mcrit                :',F7.3/
     &         1X,72('='))
C
 2060 FORMAT(/' Enter Mach #s to plot or <cr> for standard set')
C
      END ! AEROEDIT



      SUBROUTINE SETIAERO
C--------------------------------------------------
C     Sets up indices referring to aero section for 
C     each radial station
C--------------------------------------------------
      INCLUDE 'XROTOR.INC'
C
C--- Find lower index of aero data sections XIAERO(N) bounding XI(IS)
      DO I=1, II
        IAERO(I) = 1
        DO N = 1, NAERO
         IF(XIAERO(N).LE.XI(I)) THEN
           IAERO(I) = N
         ENDIF
        END DO
      END DO
      RETURN
      END



      SUBROUTINE GETAERO(N,XISECT,A0,CLMAX,CLMIN,
     &                   DCLDA,DCLDA_STALL,DCL_STALL,
     &                   CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C---------------------------------------------
C     Gets aero data from stored section array
C---------------------------------------------
      INCLUDE 'XROTOR.INC'
C
      IF(N.LT.1 .OR. N.GT.NAERO) THEN
        WRITE(*,*) 'Error: index of aero section out of bounds'
        RETURN
      ENDIF
C
      A0          = AERODATA( 1,N)
      CLMAX       = AERODATA( 2,N)
      CLMIN       = AERODATA( 3,N)
      DCLDA       = AERODATA( 4,N)
      DCLDA_STALL = AERODATA( 5,N)
      DCL_STALL   = AERODATA( 6,N)
      CDMIN       = AERODATA( 7,N)
      CLDMIN      = AERODATA( 8,N)
      DCDCL2      = AERODATA( 9,N)
      CMCON       = AERODATA(10,N)
      REREF       = AERODATA(11,N)
      REXP        = AERODATA(12,N)
      MCRIT       = AERODATA(13,N)
      XISECT      = XIAERO(N)
C
      RETURN
      END


      SUBROUTINE PUTAERO(N,XISECT,A0,CLMAX,CLMIN,
     &                   DCLDA,DCLDA_STALL,DCL_STALL,
     &                   CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C--------------------------------------------------------
C     Puts aero data into stored section array at index N
C--------------------------------------------------------
      INCLUDE 'XROTOR.INC'
C 
      IF(N.GT.NAX) THEN
        WRITE(*,*) 'Too many aero sections defined...'
        RETURN
      ENDIF
C
      AERODATA( 1,N) = A0
      AERODATA( 2,N) = CLMAX
      AERODATA( 3,N) = CLMIN
      AERODATA( 4,N) = DCLDA
      AERODATA( 5,N) = DCLDA_STALL
      AERODATA( 6,N) = DCL_STALL
      AERODATA( 7,N) = CDMIN  
      AERODATA( 8,N) = CLDMIN
      AERODATA( 9,N) = DCDCL2
      AERODATA(10,N) = CMCON 
      AERODATA(11,N) = REREF
      AERODATA(12,N) = REXP
      AERODATA(13,N) = MCRIT
      XIAERO(N)      = XISECT
C
      RETURN
      END



      SUBROUTINE SORTAR(NS,S,W,NDIM)
C----------------------------------------------------
C---- sort arrays by S values
C     Orders data monotonically increasing in S(i)
C----------------------------------------------------
      DIMENSION S(NS), W(NDIM,NS)
      LOGICAL DONE
C
      DO IPASS=1, 500
        DONE = .TRUE.
        DO N=1, NS-1
          NP = N+1
          IF(S(NP).LT.S(N)) THEN
           TEMP  = S(NP)
           S(NP) = S(N)
           S(N)  = TEMP
           DO L = 1, NDIM
             TEMP    = W(L,NP)
             W(L,NP) = W(L,N)
             W(L,N)  = TEMP
           END DO
           DONE = .FALSE.
          ENDIF
        END DO
        IF(DONE) GO TO 10
      END DO
      STOP 'SORTAR failed'
C
 10   RETURN
      END ! SORTAR


C*************************************************************************
C  Interpolated aero section properties functions
C  These routines implement a functional representation of the 
C  blade aero properties (CL,CD,CM) vs ALFA
C*************************************************************************


      SUBROUTINE GETCLCDCM(IS,ALF,W,REY,
     &                     CLIFT,CL_ALF,CL_W,
     &                     CLMAX,CLMIN,DCL_STALL,STALLF,
     &                     CDRAG,CD_ALF,CD_W,CD_REY,
     &                     CMOM,CM_AL,CM_W)
C-------------------------------------------------------------
C     CL(alpha),
C      CD(alpha), 
C       CM(alpha) interpolation function for blade at station IS
C-------------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      LOGICAL STALLF,STALLF2
C
C--- Check for installed aero data section index
      N = IAERO(IS)
      IF(N.LT.1 .OR. N.GT.NAERO) THEN
C
       IF(NAERO.GT.1) THEN
C--- Find lower index of aero data sections XIAERO(N) bounding XI(IS)
        DO N = 1, NAERO
         IF(XIAERO(N).LE.XI(IS)) THEN
cc          write(*,*) 'getcl iaero= ',N,' is= ',is,xiaero(N),xi(is)
           IAERO(IS) = N
          ELSE
           GO TO 10
         ENDIF
        END DO
        WRITE(*,*) 'Aero section not found for station ',XI(IS)
       ENDIF
C
       N = 1
       IAERO(IS) = N
      ENDIF
C
C--- Get section aero data from stored section array
 10   A0          = AERODATA( 1,N)
      CLMAX       = AERODATA( 2,N)
      CLMIN       = AERODATA( 3,N)
      DCLDA       = AERODATA( 4,N)
      DCLDA_STALL = AERODATA( 5,N)
      DCL_STALL   = AERODATA( 6,N)
      CDMIN       = AERODATA( 7,N)
      CLDMIN      = AERODATA( 8,N)
      DCDCL2      = AERODATA( 9,N)
      CMCON       = AERODATA(10,N)
      REREF       = AERODATA(11,N)
      REXP        = AERODATA(12,N)
      MCRIT       = AERODATA(13,N)
      XISECT1     = XIAERO(N)
C--- Get data for inner bounding aero section
      CALL CLCDCM(ALF,W,REY,
     &            CLIFT,CL_ALF,CL_W,STALLF,
     &            CDRAG,CD_ALF,CD_W,CD_REY,
     &            CMOM,CM_AL,CM_W,
     &            A0,CLMAX,CLMIN,DCLDA,DCLDA_STALL,DCL_STALL,
     &            CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C
C--- Check for another bounding section, if not we are done, 
C    if we have another section linearly interpolate data to station IS
      IF(N.LT.NAERO) THEN
        XISECT2 = XIAERO(N+1)
        FRAC = (XI(IS)-XISECT1)/(XISECT2-XISECT1)
        IF(FRAC.LE.0.0 .OR. FRAC.GT.1.0) THEN
cc         write(*,*) 'CL n,is,xi,frac = ',n,is,xi(is),frac
        ENDIF
C
        A0          = AERODATA( 1,N+1)
        CLMAX2      = AERODATA( 2,N+1)
        CLMIN2      = AERODATA( 3,N+1)
        DCLDA       = AERODATA( 4,N+1)
        DCLDA_STALL = AERODATA( 5,N+1)
        DCL_STALL2  = AERODATA( 6,N+1)
        CDMIN       = AERODATA( 7,N+1)
        CLDMIN      = AERODATA( 8,N+1)
        DCDCL2      = AERODATA( 9,N+1)
        CMCON       = AERODATA(10,N+1)
        REREF       = AERODATA(11,N+1)
        REXP        = AERODATA(12,N+1)
        MCRIT       = AERODATA(13,N+1)
C--- Get data for outer bounding aero section
        CALL CLCDCM(ALF,W,REY,
     &              CLIFT2,CL_ALF2,CL_W2,STALLF2,
     &              CDRAG2,CD_ALF2,CD_W2,CD_REY2,
     &              CMOM2,CM_AL2,CM_W2,
     &              A0,CLMAX2,CLMIN2,DCLDA,DCLDA_STALL,DCL_STALL2,
     &              CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C--- Interpolate aero data to blade station
        STALLF = STALLF .OR. STALLF2
        CLIFT  = (1.0-FRAC)*CLIFT  + FRAC*CLIFT2
        CL_ALF = (1.0-FRAC)*CL_ALF + FRAC*CL_ALF2
        CL_W   = (1.0-FRAC)*CL_W   + FRAC*CL_W2
        CLMAX  = (1.0-FRAC)*CLMAX  + FRAC*CLMAX2
        CLMIN  = (1.0-FRAC)*CLMIN  + FRAC*CLMIN2
        DCL_STALL = (1.0-FRAC)*DCL_STALL + FRAC*DCL_STALL2
C
        CMOM   = (1.0-FRAC)*CMOM   + FRAC*CMOM2
        CM_AL  = (1.0-FRAC)*CM_AL  + FRAC*CM_AL2
        CM_W   = (1.0-FRAC)*CM_W   + FRAC*CM_W2
C
        CDRAG  = (1.0-FRAC)*CDRAG  + FRAC*CDRAG2
        CD_ALF = (1.0-FRAC)*CD_ALF + FRAC*CD_ALF2
        CD_W   = (1.0-FRAC)*CD_W   + FRAC*CD_W2
        CD_REY = (1.0-FRAC)*CD_REY + FRAC*CD_REY2
      ENDIF
C
      RETURN
      END



      SUBROUTINE GETALF(IS,CLIFT,W,ALF,ALF_CL,ALF_W,STALLF)
C------------------------------------------------------------
C     Inverse alpha(CL) function 
C     Uses Newton-Raphson iteration to get ALF from CL function
C------------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      LOGICAL STALLF
      DATA NITER / 10 /
      DATA EPS   / 1.0E-5 /
C
      STALLF = .FALSE.
C
C---HHY had to set A0 to first aero section as A0 is now section property
      A0  = AERODATA(1,1)
      REY = 0.0
C
      ALF = A0 
      DO ITER=1, NITER
        CALL GETCLCDCM(IS,ALF,W,REY,
     &                 CLTEMP,CL_ALF,CL_W,
     &                 CLMAX,CLMIN,DCL_STALL,STALLF,
     &                 CDRAG,CD_ALF,CD_W,CD_REY,
     &                 CMOM,CM_AL,CM_W)
cc      IF(STALLF) GO TO 20
        DALF = -(CLTEMP-CLIFT)/CL_ALF
        ALF = ALF + DALF
        ALF_CL =   1.0/CL_ALF
        ALF_W  = -CL_W/CL_ALF
        IF(ABS(DALF).LT.EPS) RETURN
      END DO
C
   20 WRITE(*,*) 'GETALF: alpha(CL) function inversion failed'
c      write(*,*) 'is,clift  ',is,clift
c      write(*,*) 'abs(dalf) ',abs(dalf)
c      write(*,*) 'cl_alf    ',cl_alf
C
      RETURN
      END ! GETALF



C*************************************************************************
C  Basic aero section properties functions
C  These routines implement a functional representation of the 
C  blade section aero properties (CL,CD,CM) vs ALFA
C*************************************************************************

      SUBROUTINE CLCDCM(ALF,W,REY,
     &                  CLIFT,CL_ALF,CL_W,STALLF,
     &                  CDRAG,CD_ALF,CD_W,CD_REY,
     &                  CMOM,CM_AL,CM_W,
     &                  A0,CLMAX,CLMIN,DCLDA,DCLDA_STALL,DCL_STALL,
     &                  CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C------------------------------------------------------------
C     CL(alpha) function
C     Note that in addition to setting CLIFT and its derivatives
C     CLMAX and CLMIN (+ and - stall CL's) are set in this routine
C     In the compressible range the stall CL is reduced by a factor
C     proportional to Mcrit-Mach.  Stall limiting for compressible 
C     cases begins when the compressible drag added CDC > CDMstall
C------------------------------------------------------------
C     CD(alpha) function - presently CD is assumed to be a sum
C     of profile drag + stall drag + compressibility drag
C     In the linear lift range drag is CD0 + quadratic function of CL-CLDMIN
C     In + or - stall an additional drag is added that is proportional
C     to the extent of lift reduction from the linear lift value.
C     Compressible drag is based on adding drag proportional to 
C     (Mach-Mcrit_eff)^MEXP
C------------------------------------------------------------
C     CM(alpha) function - presently CM is assumed constant,
C     varying only with Mach by Prandtl-Glauert scaling
C------------------------------------------------------------
C
      INCLUDE 'XROTOR.INC'
      LOGICAL STALLF
      DOUBLE PRECISION ECMIN, ECMAX
C
C---- Factors for compressibility drag model, HHY 10/23/00
C     Mcrit is set by user
C     Effective Mcrit is Mcrit_eff = Mcrit - CLMFACTOR*(CL-CLDmin) - DMDD
C     DMDD is the delta Mach to get CD=CDMDD (usually 0.0020)
C     Compressible drag is CDC = CDMFACTOR*(Mach-Mcrit_eff)^MEXP
C     CDMstall is the drag at which compressible stall begins
C
      CDMFACTOR = 10.0
      CLMFACTOR =  0.25
      MEXP      =  3.0
      CDMDD     =  0.0020
      CDMSTALL  =  0.1000
C
C---- Prandtl-Glauert compressibility factor
      MSQ   =   W*W*VEL**2/VSO**2
      MSQ_W = 2.0*W*VEL**2/VSO**2
      IF(MSQ.GE.1.0) THEN
       WRITE(*,*)
     &  'CLFUNC: Local Mach number limited to 0.99, was ', MSQ
       MSQ = 0.99
       MSQ_W = 0.
      ENDIF
      PG = 1.0 / SQRT(1.0 - MSQ)
      PG_W = 0.5*MSQ_W * PG**3
C
C---- Mach number and dependence on velocity
      MACH = SQRT(MSQ)
      MACH_W = 0.0
      IF(MACH.NE.0.0) MACH_W = 0.5*MSQ_W/MACH 
C
C
C------------------------------------------------------------
C--- Generate CL from dCL/dAlpha and Prandtl-Glauert scaling
      CLA     = DCLDA*PG  *(ALF-A0)
      CLA_ALF = DCLDA*PG
      CLA_W   = DCLDA*PG_W*(ALF-A0)
C
C--- Effective CLmax is limited by Mach effects
C    reduces CLmax to match the CL of onset of serious compressible drag
      CLMX = CLMAX
      CLMN = CLMIN
      DMSTALL  = (CDMSTALL/CDMFACTOR)**(1.0/MEXP)
      CLMAXM = MAX(0.0, (MCRIT+DMSTALL-MACH)/CLMFACTOR) + CLDMIN
      CLMAX  = MIN(CLMAX,CLMAXM)
      CLMINM = MIN(0.0,-(MCRIT+DMSTALL-MACH)/CLMFACTOR) + CLDMIN
      CLMIN  = MAX(CLMIN,CLMINM)
C
C--- CL limiter function (turns on after +-stall 
      ECMAX = DEXP( MIN(200.0D0,DBLE((CLA-CLMAX)/DCL_STALL)) )
      ECMIN = DEXP( MIN(200.0D0,DBLE((CLMIN-CLA)/DCL_STALL)) )
      CLLIM = DCL_STALL * DLOG( (1.0D0+ECMAX)/(1.0D0+ECMIN) )
      CLLIM_CLA = ECMAX/(1.0+ECMAX) + ECMIN/(1.0+ECMIN)
c
c      if(CLLIM.GT.0.001) then
c      write(*,999) 'cla,cllim,ecmax,ecmin ',cla,cllim,ecmax,ecmin
c      endif
c 999  format(a,2(1x,f10.6),3(1x,d12.6))
C
C--- Subtract off a (nearly unity) fraction of the limited CL function
C    This sets the dCL/dAlpha in the stalled regions to 1-FSTALL of that
C    in the linear lift range
      FSTALL = DCLDA_STALL/DCLDA
      CLIFT  = CLA     - (1.0-FSTALL)*CLLIM
      CL_ALF = CLA_ALF - (1.0-FSTALL)*CLLIM_CLA*CLA_ALF
      CL_W   = CLA_W   - (1.0-FSTALL)*CLLIM_CLA*CLA_W
C
      STALLF = .FALSE.
      IF(CLIFT.GT.CLMAX) STALLF = .TRUE.
      IF(CLIFT.LT.CLMIN) STALLF = .TRUE.
C
C
C------------------------------------------------------------
C--- CM from CMCON and Prandtl-Glauert scaling
      CMOM  = PG*CMCON
      CM_AL = 0.0
      CM_W  = PG_W*CMCON
C
C
C------------------------------------------------------------
C--- CD from profile drag, stall drag and compressibility drag 
C
C---- Reynolds number scaling factor
      IF(REY.LE.0) THEN
       RCORR = 1.0
       RCORR_REY = 0.0
      ELSE
       RCORR     = (REY/REREF)**REXP
       RCORR_REY =  REXP/REY
      ENDIF
C
C--- In the basic linear lift range drag is a function of lift
C    CD = CD0 (constant) + quadratic with CL)
      CDRAG  = (CDMIN + DCDCL2*(CLIFT-CLDMIN)**2    ) * RCORR
      CD_ALF = (    2.0*DCDCL2*(CLIFT-CLDMIN)*CL_ALF) * RCORR
      CD_W   = (    2.0*DCDCL2*(CLIFT-CLDMIN)*CL_W  ) * RCORR
      CD_REY = CDRAG*RCORR_REY
C
C--- Post-stall drag added
      FSTALL = DCLDA_STALL/DCLDA
      DCDX    = (1.0-FSTALL)*CLLIM/(PG*DCLDA)
c      write(*,*) 'cla,cllim,fstall,pg,dclda ',cla,cllim,fstall,pg,dclda
      DCD     = 2.0* DCDX**2
      DCD_ALF = 4.0* DCDX * 
     &         (1.0-FSTALL)*CLLIM_CLA*CLA_ALF/(PG*DCLDA)
      DCD_W = 4.0* DCDX * 
     &       ( (1.0-FSTALL)*CLLIM_CLA*CLA_W/(PG*DCLDA) - DCD/PG*PG_W )
c      write(*,*) 'alf,cl,dcd,dcd_alf,dcd_w ',alf,clift,dcd,dcd_alf,dcd_w
C
C--- Compressibility drag (accounts for drag rise above Mcrit with CL effects
C    CDC is a function of a scaling factor*(M-Mcrit(CL))**MEXP
C    DMDD is the Mach difference corresponding to CD rise of CDMDD at MCRIT
      DMDD = (CDMDD/CDMFACTOR)**(1.0/MEXP)
      CRITMACH = MCRIT-CLMFACTOR*ABS(CLIFT-CLDMIN) - DMDD
      CRITMACH_ALF  = -CLMFACTOR*ABS(CL_ALF)
      CRITMACH_W    = -CLMFACTOR*ABS(CL_W)
      IF(MACH.LT.CRITMACH) THEN
       CDC     = 0.0
       CDC_ALF = 0.0
       CDC_W   = 0.0
      ELSE
       CDC = CDMFACTOR*(MACH-CRITMACH)**MEXP
       CDC_W   = MEXP*MACH_W*CDC/MACH - MEXP*CRITMACH_W  *CDC/CRITMACH
       CDC_ALF =                      - MEXP*CRITMACH_ALF*CDC/CRITMACH
      ENDIF
c      write(*,*) 'critmach,mach ',critmach,mach
c      write(*,*) 'cdc,cdc_w,cdc_alf ',cdc,cdc_w,cdc_alf
C
      FAC   = 1.0
      FAC_W = 0.0
C--- Although test data does not show profile drag increases due to Mach # 
C    you could use something like this to add increase drag by Prandtl-Glauert
C    (or any function you choose) 
cc      FAC   = PG
cc      FAC_W = PG_W
C--- Total drag terms
      CDRAG  = FAC*CDRAG              + DCD     + CDC
      CD_ALF = FAC*CD_ALF             + DCD_ALF + CDC_ALF
      CD_W   = FAC*CD_W + FAC_W*CDRAG + DCD_W   + CDC_W
      CD_REY = FAC*CD_REY
C
      RETURN
      END ! CLCDCM



      SUBROUTINE AEROPLT(NSEC,XISECT,NMACH,AMACH,
     &                  A0,CLMAX,CLMIN,DCLDA,DCLDA_STALL,DCL_STALL,
     &                  CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
C------------------------------------------------------------
C     Plots section characteristics based on parametric model
C------------------------------------------------------------
      INCLUDE 'XROTOR.INC'
      PARAMETER ( NTMP=301 )
      LOGICAL LDUMMY, STALLF
      EXTERNAL PLCHAR,PLMATH
C
      DIMENSION ATMP(NTMP), XTMP(NTMP), YTMP(NTMP)
      DIMENSION XLIN(2), YLIN(2), AMACH(NMACH)
C
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
C---- plot scale factor and aspect ratio
      PLFAC = 0.8
      PLPAR = PAR
C
C---- character size for axis numbers, labels
      CS  = CSIZE*0.8
      CSL = CSIZE*1.0
C
C---- CD, CL, alpha axis annotation increments
      DCD = 0.002
      DCL = 0.2
      DAL = 2.0
C
      DALFLIN = (CLMAX/DCLDA - CLMIN/DCLDA 
     &          + 8.0*DCL_STALL/DCLDA) * 180.0/PI
      IF(CLMAX-CLMIN .GT.  2.01)  DCL =  0.5
      IF(DALFLIN     .GT. 10.01)  DAL =  5.0
      IF(DALFLIN     .GT. 20.01)  DAL = 10.0
C
      IF(5.0*CDMIN   .GT. 0.016)  DCD = 0.005
      IF(5.0*CDMIN   .GT. 0.040)  DCD = 0.01
      IF(5.0*CDMIN   .GT. 0.080)  DCD = 0.02
      IF(5.0*CDMIN   .GT. 0.160)  DCD = 0.05
      IF(5.0*CDMIN   .GT. 0.400)  DCD = 0.10
C
C---- set plot limits
      CLMIN0 = DCL * AINT( MIN(CLMIN,0.0)/DCL - 0.5 )
      CLMAX0 = DCL * AINT( MAX(CLMAX,0.0)/DCL + 0.5 )
C
      CDMIN0 = 0.0
      CDMAX0 = DCD * AINT( 5.0*CDMIN/DCD + 0.5 )
      CDMAX0 = MAX( CDMAX0 , 0.001 )
C
      ALFMI = (CLMIN-8.0*DCL_STALL)/DCLDA
      ALFMA = (CLMAX+8.0*DCL_STALL)/DCLDA
      ALMIN0 = DAL * AINT( (ALFMI-0.02)*180.0/PI / DAL - 0.5 )
      ALMAX0 = DAL * AINT( (ALFMA+0.02)*180.0/PI / DAL + 0.5 )
C
C---- set CL, CD, alpha scaling factors
      CLWT = PLPAR/(CLMAX0-CLMIN0)
      CDWT = 0.60 /(CDMAX0-CDMIN0)
      ALWT = 0.30 /(ALMAX0-ALMIN0)
C
C
      CALL PLTINI(SCRNFR,IPSLU,IDEV,PLFAC*SIZE,LPLOT,LLAND)
      CALL PLOTABS(1.0,0.75,-3)
C
      CALL GETCOLOR(ICOL0)
C
C---- re-origin for CD-CL plot
      CALL PLOT(0.0,-CLWT*CLMIN0,-3)
      CALL PLOT(5.0*CS,0.0,-3)
C
C---- plot case name, section # and r/R location
      CALL NEWPEN(2)
      XPLT = 0.0
      YPLT = CLWT*CLMAX0 + 1.0*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,NAME,0.0,-1)
      XPLT = 0.0
      YPLT = CLWT*CLMIN0 - 2.5*CS
      CALL PLCHAR(XPLT,YPLT,CS,'Section # ',0.0,10)
      CALL PLNUMB(999.,999.,CS, FLOAT(NSEC),0.0,-1)
      CALL PLCHAR(999.,999.,CS,'  r/R = ',0.0,8)
      CALL PLNUMB(999.,999.,CS, XISECT,0.0,4)
      CALL PLCHAR(999.,999.,CS,'  REref = ',0.0,10)
      CALL PLNUMB(999.,999.,CS, REREF,0.0,-1)
C
C---- plot CD-CL axes
      CALL NEWPEN(2)
      CALL YAXIS(0.0,CLWT*CLMIN0, CLWT*(CLMAX0-CLMIN0),
     &               CLWT*DCL,CLMIN0,DCL,CS,1)
      CALL XAXIS(CDWT*CDMIN0,0.0,-CDWT*(CDMAX0-CDMIN0),
     &               CDWT*DCD,CDMIN0,DCD,CS,3)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT( (CDMAX0-CDMIN0)/DCD + 0.01 )
       NYG = INT( (CLMAX0-CLMIN0)/DCL + 0.01 )
       CALL PLGRID(CDWT*CDMIN0,CLWT*CLMIN0, 
     &             NXG,CDWT*DCD, NYG,CLWT*DCL, LMASK2 )
      ENDIF
C
C---- legend location
      XLEG    = CDWT*CDMIN0 + 10.0*CSL
      YLEG    = CLWT*CLMAX0 +  3.5*CSL
      XLIN(1) = CDWT*CDMIN0
      XLIN(2) = CDWT*CDMIN0 +  8.0*CSL
      YLIN(1) = CLWT*CLMAX0 +  3.5*CSL
      YLIN(2) = CLWT*CLMAX0 +  3.5*CSL
C
C---- CL label
      CALL NEWPEN(3)
      XPLT = -3.0*CSL
      YPLT = CLWT*(CLMAX0-1.5*DCL) - 0.3*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,'c',0.0,1)
      CALL PLSUBS(XPLT,YPLT,CSL,'V',0.0,1,PLMATH)
C
C---- CD label
      CALL NEWPEN(2)
      XPLT = CDWT*(CDMAX0-1.5*DCD) - 0.7*CSL
      YPLT = -2.8*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,'c',0.0,1)
      CALL PLSUBS(XPLT,YPLT,CSL,'d',0.0,1,PLCHAR)
C
C---- plot CL-CD polar curve 
      CALL NEWPEN(3)
      DALF = (ALMAX0-ALMIN0)/FLOAT(NTMP-1)
      DO IMACH=1, NMACH
C--- Limit MACH to 0.95 for sanity
        MACH = MIN(0.95,AMACH(IMACH))
        MSQ = MACH**2
        W = VSO*SQRT(MSQ) / VEL
        PG = 1.0 / SQRT(1.0 - MSQ)
C
        CALL NEWCOLOR(2+IMACH)
        ILIN = IMACH
C
        DO N=1, NTMP
C-------- set alpha
          ALFA = (ALMIN0 + DALF*FLOAT(N-1)) * PI/180.0
C-------- set corresponding CL and CD
          CLMX = CLMAX
          CLMN = CLMIN
          CALL CLCDCM(ALFA,W,REREF,
     &                CLIFT,CL_ALF,CL_W,STALLF,
     &                CDRAG,CD_ALF,CD_W,CD_REY,
     &                CMOM,CM_AL,CM_W,
     &                A0,CLMX,CLMN,DCLDA,DCLDA_STALL,DCL_STALL,
     &                CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
          YTMP(N) = CLIFT
          XTMP(N) = CDRAG
        END DO
        N = NTMP
        CALL CHKLIM(N,N1,N2,XTMP,1.1*CDMAX0)
        CALL XYLINE(N2-N1+1,XTMP(N1),YTMP(N1),0.0,CDWT,0.0,CLWT,ILIN)
C------ plot legend
        DELY = 2.5*CSL*FLOAT(IMACH-1)
        CALL XYLINE(2,XLIN,YLIN,0.0,1.0,-DELY,1.0,ILIN)
        CALL PLCHAR(XLEG,YLEG+DELY,CSL,'M = ',0.0,4)
        CALL PLNUMB(999.,999.,CSL, MACH,0.0,2)
C
      END DO
      CALL NEWCOLOR(ICOL0)
C
C---- reset origin for alpha-CL plot
      CALL PLOT( CDWT*CDMAX0,0.0,-3)
      CALL PLOT( 0.10 , 0.0, -3)
      CALL PLOT(-ALWT*ALMIN0,0.0,-3)
C
C---- plot alpha-CL axes
      CALL NEWPEN(2)
      CALL YAXIS(0.0,CLWT*CLMIN0,-CLWT*(CLMAX0-CLMIN0),
     &               CLWT*DCL,CLMIN0,DCL,CS, 1)
      CALL XAXIS(ALWT*ALMIN0,0.0,-ALWT*(ALMAX0-ALMIN0),
     &               ALWT*DAL,ALMIN0,DAL,CS,-1)
      IF(LGRID) THEN
       CALL NEWPEN(1)
       NXG = INT( (ALMAX0-ALMIN0)/DAL + 0.01 )
       NYG = INT( (CLMAX0-CLMIN0)/DCL + 0.01 )
       CALL PLGRID(ALWT*ALMIN0,CLWT*CLMIN0, 
     &              NXG,ALWT*DAL, NYG,CLWT*DCL, LMASK2 )
      ENDIF
C
C---- CL label
      CALL NEWPEN(3)
      XPLT = -3.0*CSL
      YPLT = CLWT*(CLMAX0-1.5*DCL) - 0.3*CSL
      CALL PLCHAR(XPLT,YPLT,CSL,'c',0.0,1)
      CALL PLSUBS(XPLT,YPLT,CSL,'V',0.0,1,PLMATH)
C
C---- alpha label
      CALL NEWPEN(3)
      XPLT = ALWT*(ALMAX0-0.5*DAL) - 0.8*CSL
      YPLT = -2.8*CSL
      CALL PLMATH(XPLT,YPLT,1.4*CS,'a"',0.0,2)
C
C---- plot alpha-CL curves
      CALL NEWPEN(3)
      DALF = (ALMAX0-ALMIN0)/FLOAT(NTMP-1)
      DO IMACH=1, NMACH
C--- Limit MACH to 0.95 for sanity
        MACH = MIN(0.95,AMACH(IMACH))
        MSQ = MACH**2
        W = VSO*SQRT(MSQ) / VEL
        PG = 1.0 / SQRT(1.0 - MSQ)
C
        CALL NEWCOLOR(2+IMACH)
        ILIN = IMACH
C
        DO N=1, NTMP
C-------- set alpha
          ALFA = (ALMIN0 + DALF*FLOAT(N-1)) * PI/180.0
C-------- set corresponding CL and CD
          CLMX = CLMAX
          CLMN = CLMIN
          CALL CLCDCM(ALFA,W,REREF,
     &                CLIFT,CL_ALF,CL_W,STALLF,
     &                CDRAG,CD_ALF,CD_W,CD_REY,
     &                CMOM,CM_AL,CM_W,
     &                A0,CLMX,CLMN,DCLDA,DCLDA_STALL,DCL_STALL,
     &                CDMIN,CLDMIN,DCDCL2,CMCON,MCRIT,REREF,REXP)
          XTMP(N) = ALFA
          YTMP(N) = CLIFT
       END DO
        N = NTMP
        CALL CHKLIM(N,N1,N2,XTMP,1.1*ALMAX0*PI/180.)
        CALL XYLINE(N2-N1+1,XTMP(N1),YTMP(N1),0.0,ALWT*180.0/PI,0.0,
     &              CLWT,ILIN)
      END DO
      CALL NEWCOLOR(ICOL0)
C
      CALL PLFLUSH
C
C---- set factors and offsets for CL(CD) plot
      XYOFF(1) = 0.
      XYOFF(2) = 0.
      XYFAC(1) = CDWT
      XYFAC(2) = CLWT
C
      CALL PLOTABS(1.0,0.5,-3)
      CALL PLOT(0.0,-CLWT*CLMIN0,-3)
      CALL PLOT(5.0*CS,0.0,-3)
C
      RETURN
      END ! AERPLT


      SUBROUTINE CHKLIM(N,NSTRT,NEND,F,FMAX)
C--- Get starting and end index for array values F(i) < FMAX
      DIMENSION F(N)
      NSTRT = 1
      NEND  = N
C--- Look for first point where F(i)<FMAX
      DO I=1,N
          IF(F(I).LT.FMAX) GO TO 10
      END DO
 10   NSTRT = MAX(I-1,1)
C--- Look for last point where F(i)<FMAX
      DO I=N,1,-1
          IF(F(I).LT.FMAX) GO TO 20
      END DO
 20   NEND = MIN(I+1,N)
C
      RETURN
      END



