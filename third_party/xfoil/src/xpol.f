C***********************************************************************
C    Module:  xpol.f
C 
C    Copyright (C) 2000 Mark Drela 
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


      SUBROUTINE PLRSET(IP)
C--------------------------------------------------------------
C     Selects slot IP for saving polar.
C     Resets all parameters if necessary.
C--------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      LOGICAL ERROR
C
      IF(IP.LE.0) THEN
C----- invalid polar index
       RETURN
C
      ELSEIF(IP.GE.1 .AND. IP.LE.NPOL) THEN
       WRITE(*,*)
       WRITE(*,*) 'Existing stored polar is chosen for appending...'
       NIPOL = NIPOL0
       IF(LCMINP) THEN
        NIPOL = NIPOL + 1
        IPOL(NIPOL) = IMC
       ENDIF
       IF(LHMOMP) THEN
        NIPOL = NIPOL + 1
        IPOL(NIPOL) = ICH
       ENDIF
       CALL POLWRIT(6,' ',ERROR, .TRUE.,
     &              NAX, 1,NAPOL(IP), CPOL(1,1,IP),IPOL,NIPOL,
     &              REYNP1(IP),MACHP1(IP),ACRITP(1,IP),XSTRIPP(1,IP),
     &              PTRATP(IP),ETAPP(IP),
     &              NAMEPOL(IP), IRETYP(IP),IMATYP(IP),
     &              ISX,1,CPOLSD(1,1,1,IP), JPOL,NJPOL,
     &              'XFOIL',VERSION, .FALSE.)
       NIPOL = NIPOL0
C
C----- check if geometries differ...
       IF(N.NE.NXYPOL(IP)) GO TO 10
       SIZREF = S(N) - S(1)
       DO I = 1, N
         DSQ = (X(I)-CPOLXY(I,1,IP))**2 + (Y(I)-CPOLXY(I,2,IP))**2
         DSFRAC = SQRT(DSQ) / SIZREF
         IF(DSFRAC .GT. 0.00001) GO TO 10
       ENDDO
       GO TO 20
C
 10    WRITE(*,*) 'Current airfoil differs from airfoil of stored polar'
       WRITE(*,1100)
 1100  FORMAT(
     & /'   - - - - - - - - - - - - - - - - - - - - - - - - - - - -'
     & /'    0  abort polar accumulation'
     & /'    1  compute with current airfoil'
     & /'    2  compute with stored  airfoil',
     &          ' (overwrite current airfoil)')
       CALL ASKI('   Select action^', IOPT)
       IF(IOPT.EQ.0) THEN
        IP = 0
        RETURN
       ELSEIF(IOPT.EQ.1) THEN
        CONTINUE
       ELSEIF(IOPT.EQ.2) THEN
        CALL APCOPY(IP)
       ENDIF
C
 20    CONTINUE
       WRITE(*,*) 
       WRITE(*,*) 'Setting current parameters to those of stored polar'
C
       NAME = NAMEPOL(IP)
       CALL STRIP(NAME,NNAME)
C
       RETYP = IRETYP(IP)
       MATYP = IMATYP(IP)
C
       MINF1 = MACHP1(IP)
       REINF1 = REYNP1(IP)
C
       ACRIT(1) = ACRITP(1,IP)
       ACRIT(2) = ACRITP(2,IP)
C
       XSTRIP(1) = XSTRIPP(1,IP)
       XSTRIP(2) = XSTRIPP(2,IP)
C
      ELSE
C----- new polar slot is chosen
       NPOL = IP
C
       NAPOL(IP) = 0
C
       NAMEPOL(IP) = NAME
       IRETYP(IP) = RETYP
       IMATYP(IP) = MATYP
C
       IF(LVISC) THEN
        REYNP1(IP) = REINF1
       ELSE
        REYNP1(IP) = 0.
       ENDIF
       MACHP1(IP) = MINF1
C
       ACRITP(1,IP) = ACRIT(1)
       ACRITP(2,IP) = ACRIT(2)
C
       XSTRIPP(1,IP) = XSTRIP(1)
       XSTRIPP(2,IP) = XSTRIP(2)
C
       NXYPOL(IP) = N
       DO I = 1, N
         CPOLXY(I,1,IP) = X(I)
         CPOLXY(I,2,IP) = Y(I)
       ENDDO
C
       WRITE(*,2100) IP, NAMEPOL(IP)
 2100  FORMAT(/' Polar', I3, ' newly created for accumulation'
     &        /' Airfoil archived with polar: ', A)
      ENDIF
C
      END ! PLRSET


      SUBROUTINE APCOPY(IP)
      INCLUDE 'XFOIL.INC'
C
      NOLD = N

      N = NXYPOL(IP)
      DO I = 1, N
        X(I) = CPOLXY(I,1,IP)
        Y(I) = CPOLXY(I,2,IP)
      ENDDO
      NAME = NAMEPOL(IP)
C
      CALL SCALC(X,Y,S,N)
      CALL SEGSPL(X,XP,S,N)
      CALL SEGSPL(Y,YP,S,N)
      CALL NCALC(X,Y,S,N,NX,NY)
      CALL LEFIND(SLE,X,XP,Y,YP,S,N)
      XLE = SEVAL(SLE,X,XP,S,N)
      YLE = SEVAL(SLE,Y,YP,S,N)
      XTE = 0.5*(X(1)+X(N))
      YTE = 0.5*(Y(1)+Y(N))
      CHORD  = SQRT( (XTE-XLE)**2 + (YTE-YLE)**2 )
      CALL TECALC
      CALL APCALC
C
      LGAMU = .FALSE.
      LQINU = .FALSE.
      LWAKE = .FALSE.
      LQAIJ = .FALSE.
      LADIJ = .FALSE.
      LWDIJ = .FALSE.
      LIPAN = .FALSE.
      LVCONV = .FALSE.
      LSCINI = .FALSE.
      IF(NOLD.NE.N) LBLINI = .FALSE.
C
      RETURN
      END ! APCOPY



      SUBROUTINE PLRINI(LU,IP)
C--------------------------------------------------------------
C     Checks or initializes a polar save file.
C
C     If file PFNAME(IP) exists, it is checked for consistency 
C        with current parameters.  Polar saving is enabled
C        only if file parameters match current parameters.
C
C     If file PFNAME(IP) doesn't exist, a new one is set up by 
C        writing a header to it, and polar saving is enabled.
C--------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*128 LINE, LINEL, PROMPT
C
      LOGICAL NAMDIF, ERROR
C
      INTEGER NBLP(ISX,IPX)
C
      REAL RINP(IPTOT)
C
      CALL STRIP(PFNAME(IP),NPF)
      IF(NPF.EQ.0) THEN
       PROMPT =  'Enter  polar save filename'
     &        // '  OR  <return> for no file^'
      ELSE
       WRITE(*,*) 'Default polar save filename:  ', PFNAME(IP)(1:NPF)
       PROMPT =  'Enter  new filename'
     &        // '  OR  "none"'
     &        // '  OR  <return> for default^'
      ENDIF
C
      CALL ASKS(PROMPT,FNAME)
      CALL STRIP(FNAME,NFN)
C
      IF(NFN.EQ.0) THEN
       FNAME = PFNAME(IP)
       NFN = NPF
      ELSEIF(INDEX('NONEnone',FNAME(1:4)).NE.0) THEN
       NFN = 0
      ENDIF
C
      IF(NFN.EQ.0) THEN
       LPFILE = .FALSE.
       WRITE(*,*)
       WRITE(*,*) 'Polar save file will NOT be written'
       RETURN
      ENDIF
C
C---- no valid file yet
      LPFILE = .FALSE.
C
C---- try reading the polar file to see if it exists
      OPEN(LU,FILE=FNAME,STATUS='OLD',ERR=60)
      CALL POLREAD(LU,' ',ERROR,
     &             NAX,NAPOL(IP),CPOL(1,1,IP), 
     &             REYNP1(IP),MACHP1(IP),ACRITP(1,IP),XSTRIPP(1,IP),
     &             PTRATP(IP),ETAPP(IP),
     &             NAMEPOL(IP),IRETYP(IP),IMATYP(IP),
     &             ISX,NBLP(1,IP),CPOLSD(1,1,1,IP),
     &             CODEPOL(IP),VERSPOL(IP) )
      IF(ERROR) GO TO 90
      CLOSE(LU)
      PFNAME(IP) = FNAME
C
      CALL STRIP(NAMEPOL(IP),NNAMEP)
C
C---- check to see if the names are different
      IF(NNAME .NE. NNAMEP) THEN
        NAMDIF = .TRUE.
      ELSE
        NAMDIF = .FALSE.
        DO K=1, NNAME
          IF(NAME(K:K).NE.NAMEPOL(IP)(K:K)) NAMDIF = .TRUE.
        ENDDO
      ENDIF
C
C---- check if the polar save file is for the same airfoil and conditions
      IF(NAMDIF                   .OR.
     &   REYNP1(IP) .NE. REINF1   .OR.
     &   MACHP1(IP) .NE. MINF1    .OR.
     &   IRETYP(IP) .NE. RETYP    .OR.
     &   IMATYP(IP) .NE. MATYP    .OR.
     &   ACRITP(1,IP) .NE. ACRIT(1)    .OR.
     &   ACRITP(2,IP) .NE. ACRIT(2)    .OR.
     &   XSTRIPP(1,IP) .NE. XSTRIP(1)  .OR.
     &   XSTRIPP(2,IP) .NE. XSTRIP(2)       ) THEN
C
       WRITE(*,6600)  NAME,     NAMEPOL(IP)   ,
     &                REINF1,   REYNP1(IP)   ,
     &                MINF1,    MACHP1(IP)   ,
     &                RETYP,    IRETYP(IP)   ,
     &                MATYP,    IMATYP(IP)   ,
     &                ACRIT(1), ACRITP(1,IP) ,
     &                ACRIT(2), ACRITP(2,IP) ,
     &                XSTRIP(1),XSTRIPP(1,IP),
     &                XSTRIP(2),XSTRIPP(2,IP)
C
 6600  FORMAT(
     & /'               Current                         Save file'
     & /'           ------------------              ------------------'
     & /' name  :   ', A    ,      A
     & /' Re    :   ', F12.0, 20X, F12.0
     & /' Mach  :   ', F12.4, 20X, F12.4
     & /' Retyp :   ', I7   , 25X, I7
     & /' Matyp :   ', I7   , 25X, I7
     & /' NcritT:   ', F12.4, 20X, F12.4
     & /' NcritB:   ', F12.4, 20X, F12.4
     & /' xtr T :   ', F12.4, 20X, F12.4
     & /' xtr B :   ', F12.4, 20X, F12.4 )
C
       WRITE(*,*)
       WRITE(*,*)
     &    'Current parameters different from old save file values.'
       CALL ASKL
     &   ('Set current parameters to old save file values ?^',OK)
C
       IF(OK) THEN
        NAME    = NAMEPOL(IP)
        NNAME   = NNAMEP
        REINF1  = REYNP1(IP)
        MINF1   = MACHP1(IP)
        RETYP   = IRETYP(IP)
        MATYP   = IMATYP(IP)
        ACRIT(1) = ACRITP(1,IP)
        ACRIT(2) = ACRITP(2,IP)
        XSTRIP(1) = XSTRIPP(1,IP)
        XSTRIP(2) = XSTRIPP(2,IP)
       ELSE
        WRITE(*,*)
        WRITE(*,*) 'Old polar save file NOT available for appending'
        RETURN
       ENDIF
      ENDIF
C
C---- display polar save file just read in
      WRITE(*,*)
      WRITE(*,*) 'Old polar save file read in ...'
      CALL POLWRIT(6,' ',ERROR, .TRUE.,
     &             NAX, 1,NAPOL(IP), CPOL(1,1,IP), IPOL,NIPOL,
     &             REYNP1(IP),MACHP1(IP),ACRITP(1,IP),XSTRIPP(1,IP),
     &             PTRATP(IP),ETAPP(IP),
     &             NAMEPOL(IP), IRETYP(IP),IMATYP(IP),
     &             ISX,1,CPOLSD(1,1,1,IP), JPOL,NJPOL,
     &             CODEPOL(IP),VERSPOL(IP), .FALSE. )
C
C---- enable writing to the save file
      LPFILE = .TRUE.
      WRITE(*,*)
      WRITE(*,*) 'Old polar save file available for appending'
      RETURN
C
C
C---- the polar save file doesn't exist, so write new header
   60 CONTINUE
      NIPOL = NIPOL0
      IF(LCMINP) THEN
       NIPOL = NIPOL + 1
       IPOL(NIPOL) = IMC
      ENDIF
      IF(LHMOMP) THEN
       NIPOL = NIPOL + 1
       IPOL(NIPOL) = ICH
      ENDIF
C
      OPEN(LU,FILE=FNAME,STATUS='NEW',ERR=80)
      IA1 = 0
      IA2 = -1
      CALL POLWRIT(LU,' ',ERROR, .TRUE.,
     &             NAX, IA1,IA2, CPOL(1,1,IP), IPOL,NIPOL,
     &             REYNP1(IP),MACHP1(IP),ACRITP(1,IP),XSTRIPP(1,IP),
     &             PTRATP(IP),ETAPP(IP),
     &             NAMEPOL(IP),IRETYP(IP),IMATYP(IP),
     &             ISX,1,CPOLSD(1,1,1,IP), JPOL,NJPOL,
     &             'XFOIL',VERSION, .FALSE. )
      CLOSE(LU)
      PFNAME(IP) = FNAME
C
      NIPOL = NIPOL0
C
C---- enable writing to the save file
      LPFILE = .TRUE.
      WRITE(*,*)
      WRITE(*,*) 'New polar save file available'
      RETURN
C
C---- the polar save file doesn't exist, so write new header
   80 WRITE(*,*) 'New polar save file OPEN error'
      RETURN
C
C---- READ error trap
   90 WRITE(*,*) 'Old polar save file READ error'
      CLOSE(LU)
      RETURN
C
C..........................................
 1000 FORMAT(A)
 1010 FORMAT(22X,A32)
 1020 FORMAT( 8X,F7.3,10X,F9.3)
 1030 FORMAT( 8X,F7.3,10X,F9.3,17X,F7.3)
      END ! PLRINI



      SUBROUTINE PLXINI(LU,IP)
C--------------------------------------------------------------
C     Checks or initializes a polar dump file.
C
C     If file PFNAMX(IP) exists, it is checked for consistency 
C        with current parameters.  Polar dumping is enabled
C        only if file parameters match current parameters.
C
C     If file PFNAMX(IP) doesn't exist, a new one is set up by 
C        writing a header to it, and polar dumping is enabled.
C--------------------------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*128 PROMPT
C
      CHARACTER*32 NAMEX
      REAL MACHX
      REAL ACRITX(ISX)
      INTEGER RETYPX, MATYPX
      LOGICAL NAMDIF
C
      CALL STRIP(PFNAMX(IP),NPF)
      IF(NPF.EQ.0) THEN
       PROMPT =  'Enter  polar dump filename'
     &        // '  OR  <return> for no file^'
      ELSE
       WRITE(*,*) 'Default polar dump filename:  ', PFNAMX(IP)(1:NPF)
       PROMPT =  'Enter  new filename'
     &        // '  OR  "none"'
     &        // '  OR  <return> for default^'
      ENDIF
C
      CALL ASKS(PROMPT,FNAME)
      CALL STRIP(FNAME,NFN)
C
      IF(INDEX('NONEnone',FNAME(1:4)).NE.0) NFN = 0
C
      IF(NFN.EQ.0) THEN
       LPFILX = .FALSE.
       WRITE(*,*)
       WRITE(*,*) 'Polar dump file will NOT be written'
       RETURN
      ENDIF
C
C---- no valid dump file yet
      LPFILX = .FALSE.
C
C---- try reading the unformatted polar dump file to see if it exists
      OPEN(LU,FILE=FNAME,
     &     STATUS='UNKNOWN',FORM='UNFORMATTED',ERR=80)
      READ(LU,ERR=90,END=60) NAMEX
C
C---- if we got to here, it exists, so read the header
      READ(LU) MACHX, REYNX, ACRITX(1), ACRITX(2)
      READ(LU) MATYPX, RETYPX
      READ(LU) IIX, ILEX, ITEX, IIBX
C
      REYNX = REYNX*1.0E6
C
C---- set polar dump file pointer at the end
   45 READ(LU,END=46) DUMMY
      GO TO 45
C
   46 CLOSE(LU)
      PFNAMX(IP) = FNAME
C
      CALL STRIP(NAMEX,NNAMEX)
C
C---- check to see if the names are different
      IF(NNAME .NE. NNAMEX) THEN
        NAMDIF = .TRUE.
      ELSE
        NAMDIF = .FALSE.
        DO 50 K=1, NNAME
          IF(NAME(K:K).NE.NAMEX(K:K)) NAMDIF = .TRUE.
   50   CONTINUE
      ENDIF
C
C---- check if the polar save file is for the same airfoil and conditions
      IF(NAMDIF               .OR.
     &   REYNX  .NE. REINF1   .OR.
     &   MACHX  .NE. MINF1    .OR.
     &   RETYPX .NE. RETYP    .OR.
     &   MATYPX .NE. MATYP    .OR.
     &   ACRITX(1) .NE. ACRIT(1) .OR.
     &   ACRITX(2) .NE. ACRIT(2)     ) THEN
C
       WRITE(*,6600) NAMEX  , NAME,
     &               REYNX  , REINF1,
     &               MACHX  , MINF1,
     &               RETYPX , RETYP,
     &               MATYPX , MATYP,
     &               ACRITX(1), ACRIT(1),
     &               ACRITX(1), ACRIT(2)
C
 6600  FORMAT(
     & /'               Dump file                       Current'
     & /'             ------------                    ------------'
     & /' name  :   ', A    ,      A
     & /' Re    :   ', F12.0, 20X, F12.0
     & /' Mach  :   ', F12.4, 20X, F12.4
     & /' Retyp :   ', I7   , 25X, I7
     & /' Matyp :   ', I7   , 25X, I7
     & /' NcritT:   ', F12.4, 20X, F12.4
     & /' NcritB:   ', F12.4, 20X, F12.4 )
C
       WRITE(*,*)
       WRITE(*,*)
     &    'Current parameters different from old dump file values.'
       CALL ASKL
     &   ('Set current parameters to old dump file values ?^',OK)
C
       IF(OK) THEN
        NAME   = NAMEX
        NNAME  = NNAMEX
        MINF1  = MACHX
        REINF1 = REYNX
        RETYP  = RETYPX
        MATYP  = MATYPX
        ACRIT(1) = ACRITX(1)
        ACRIT(2) = ACRITX(2)
       ELSE
        WRITE(*,*)
        WRITE(*,*) 'Old polar dump file NOT available for appending'
        RETURN
       ENDIF
      ENDIF
C
C---- enable writing to the save file
      LPFILX = .TRUE.
      WRITE(*,*)
      WRITE(*,*) 'Old polar dump file available for appending'
      RETURN
C
C
C---- the polar dump file doesn't exist, so write new header
   60 CONTINUE
      WRITE(LU) NAME, 'XFOIL   ', VERSION
      WRITE(LU) MINF1, REINF1/1.0E6, ACRIT(1), ACRIT(2)
      WRITE(LU) MATYP, RETYP
      WRITE(LU) 0, 0, 0, N
      WRITE(LU) (X(I), Y(I), I=1, N)
C
   70 CONTINUE
C
      CLOSE(LU)
      PFNAMX(IP) = FNAME
C
C---- enable writing to the save file
      LPFILX = .TRUE.
      WRITE(*,*)
      WRITE(*,*) 'New polar dump file available'
      RETURN
C
C---- OPEN error trap
   80 WRITE(*,1080) FNAME
      RETURN
C
C---- READ error trap
   90 WRITE(*,*) 'Polar dump file READ error'
      CLOSE(LU)
      RETURN
C..........................................
 1080 FORMAT(' OPEN error on polar dump file ', A48)
      END ! PLXINI



      SUBROUTINE PLRADD(LU,IP)
      INCLUDE 'XFOIL.INC'
      LOGICAL ERROR
C
cc      WRITE(*,1000) CL, CD, CM
cc 1000 FORMAT(/' CL =', F7.3, '    Cd =', F9.5, '    Cm =', F8.4)
C
C---- add point to storage arrays
      IF(IP.EQ.0) THEN
       WRITE(*,*) 'No active polar is declared. Point not stored.'
C
      ELSE
       IF(NAPOL(IP).EQ.NAX) THEN
        WRITE(*,*) 'Polar storage arrays full. Point not stored'
C
       ELSE
        NAPOL(IP) = NAPOL(IP)+1
C
C------ store current point
        IF(LVISC) THEN
         CDTOT = CD
         CDV = CD
         RE = REINF
        ELSE
         CDTOT = 0.
         CDV = 0.
         RE = 0.
        ENDIF
C
        IA = NAPOL(IP)
        CPOL(IA,IAL,IP) = ADEG
        CPOL(IA,ICL,IP) = CL
        CPOL(IA,ICD,IP) = CDTOT
        CPOL(IA,ICM,IP) = CM
        CPOL(IA,ICP,IP) = CDP
        CPOL(IA,ICV,IP) = CDV
        CPOL(IA,IMA,IP) = MINF
        CPOL(IA,IRE,IP) = RE
        DO IS = 1, 2
          IF(LVISC) THEN
           XOCT = XOCTR(IS)
          ELSE
           XOCT = 0.
          ENDIF
          CPOLSD(IA,IS,JNC,IP) = ACRIT(IS)
          CPOLSD(IA,IS,JTP,IP) = XSTRIP(IS)
          CPOLSD(IA,IS,JTN,IP) = XOCT
          CPOLSD(IA,IS,JTI,IP) = TINDEX(IS)
        ENDDO
C
        IF(LFLAP) THEN
         CALL MHINGE
         CPOL(IA,ICH,IP) = HMOM
        ELSE
         CPOL(IA,ICH,IP) = 0.
        ENDIF
        CPOL(IA,IMC,IP) = CPMN
C
        WRITE(*,1100) IP
 1100   FORMAT(/' Point added to stored polar', I3)
       ENDIF
      ENDIF
C
C---- add point to save file
      IF(LPFILE) THEN
       NIPOL = NIPOL0
       IF(LCMINP) THEN
        NIPOL = NIPOL + 1
        IPOL(NIPOL) = IMC
       ENDIF
       IF(LHMOMP) THEN
        NIPOL = NIPOL + 1
        IPOL(NIPOL) = ICH
       ENDIF
C
       OPEN(LU,FILE=PFNAME(IP),STATUS='OLD')
       CALL BOTTOM(LU)
       IA = NAPOL(IP)
       CALL POLWRIT(LU,' ',ERROR, .FALSE.,
     &              NAX, IA,IA, CPOL(1,1,IP), IPOL,NIPOL,
     &              REYNP1(IP),MACHP1(IP),ACRITP(1,IP),XSTRIPP(1,IP),
     &              PTRATP(IP),ETAPP(IP),
     &              NAMEPOL(IP), IRETYP(IP),IMATYP(IP),
     &              ISX,1,CPOLSD(1,1,1,IP), JPOL,NJPOL,
     &              'XFOIL',VERSION, .FALSE. )
       CLOSE(LU)
       NIPOL = NIPOL0
       WRITE(*,1200) PFNAME(IP)
 1200  FORMAT(' Point written to save file  ', A48)
      ELSE
       WRITE(*,1300)
 1300  FORMAT(' Save file unspecified or not available')
      ENDIF
C
cccC---- sort polar in increasing alpha
ccc      IDSORT = IAL
ccc      CALL PLRSRT(IP,IDSORT)
C
      RETURN
      END ! PLRADD
 

      SUBROUTINE PLXADD(LU,IP)
      INCLUDE 'XFOIL.INC'
      INTEGER NSIDE(2)
C
      DIMENSION XX(IVX,2), CP(IVX,2), CF(IVX,2)
C
      IF(.NOT.LPFILX) THEN
       WRITE(*,1050)
 1050  FORMAT(' Dump file unspecified or not available')
       RETURN
      ENDIF
C
      BETA = SQRT(1.0 - MINF**2)
      BFAC = 0.5*MINF**2 / (1.0 + BETA)
C
      OPEN(LU,FILE=PFNAMX(IP),STATUS='OLD',FORM='UNFORMATTED')
      CALL BOTTOMX(LU)
C
C---- write integrated forces to unformatted dump file
      IF(LVISC) THEN
       CDTOT = CD
       XT1 = XOCTR(1)
       XT2 = XOCTR(2)
      ELSE
       CDTOT = 0.
       XT1 = 0.
       XT2 = 0.
      ENDIF
      WRITE(LU) ALFA/DTOR,CL,CDTOT,0.0,CM,XT1,XT2
C
      NSIDE(1) = IBLTE(1) + (NBL(2)-IBLTE(2))
      NSIDE(2) = NBL(2)
C
      NSIDE(1) = MAX( NSIDE(1) , 2 )
      NSIDE(2) = MAX( NSIDE(2) , 2 )
C
C---- write indexing info
      WRITE(LU) NSIDE(1), NSIDE(2), IBLTE(1), IBLTE(2)
C
      QUE = 0.5*QINF**2
C
C---- set stagnation point quantities
      IBL = 1
      XX(IBL,1) = SEVAL(SST,X,XP,S,N)
      CP(IBL,1) = 1.0 / (BETA + BFAC)
      CF(IBL,1) = 0.0
      THET(IBL,1) = 0.5*(THET(2,1) + THET(2,2))
      DSTR(IBL,1) = 0.5*(DSTR(2,1) + DSTR(2,2))
      CTAU(IBL,1) = 0.0
C
      XX(IBL,2)   = XX(IBL,1)
      CP(IBL,2)   = CP(IBL,1)
      CF(IBL,2)   = CF(IBL,1)
      THET(IBL,2) = THET(IBL,1)
      DSTR(IBL,2) = DSTR(IBL,1)
      CTAU(IBL,2) = CTAU(IBL,1)
C
C---- set BL and wake quantities
      DO 10 IS=1, 2
        DO IBL=2, NSIDE(IS)
          I = IPAN(IBL,IS)
          XX(IBL,IS) = X(I)
          CP(IBL,IS) = CPV(I)
          CF(IBL,IS) = TAU(IBL,IS) / QUE
        ENDDO
   10 CONTINUE
C
      DO IS=1, 2
        WRITE(LU) (XX(IBL,IS),CP(IBL,IS),THET(IBL,IS),DSTR(IBL,IS),
     &             CF(IBL,IS),CTAU(IBL,IS), IBL=1, NSIDE(IS))
      ENDDO
C
      CLOSE(LU)
      WRITE(*,1100) PFNAMX(IP)
 1100 FORMAT(' Point written to dump file ', A48)
      RETURN
C
      END ! PLXADD



      SUBROUTINE PLRSRT(IP,IDSORT)
      INCLUDE 'XFOIL.INC'
      DIMENSION INDX(NAX), ATMP(NAX)
C
C---- sort polar in increasing variable IDSORT
      CALL HSORT(NAPOL(IP),CPOL(1,IDSORT,IP),INDX)
C
C---- do the actual reordering
      DO ID = 1, IPTOT
        CALL ASORT(NAPOL(IP),CPOL(1,ID,IP),INDX,ATMP)
      ENDDO
      DO ID = 1, JPTOT
        DO IS = 1, 2
          CALL ASORT(NAPOL(IP),CPOLSD(1,IS,ID,IP),INDX,ATMP)
        ENDDO
      ENDDO
C
      RETURN
      END ! PLRSRT



      SUBROUTINE PLRSUM(IP1,IP2,IPACTT)
C---------------------------------------------
C     Prints summary of polars IP1..IP2
C---------------------------------------------
      INCLUDE 'XFOIL.INC'
      CHARACTER*5 CLTYP(3)
      CHARACTER*1 CACC, CFIL
C
      DATA CLTYP / '     ', '/sqCL', '/CL  ' /
C
 1100 FORMAT(1X,A,A)
      WRITE(*,*)
      WRITE(*,1100)
     & '       airfoil                    Re           Mach     ',
     & '  NcritT  NcritB  XtripT  XtripB       file'
      WRITE(*,1100)
     & '      ------------------------  ------------  ----------',
     & '  ------  ------  ------  ------    -------------------'
CCC     >  10  NACA 0012 (mod)           1.232e6/sqCL  0.781/sqCL
CCC         9.00    9.00   1.000   1.000
CCC     1234567890123456789012345678901234567890123456789012345678901234567890
C
      DO IP = IP1, IP2
        IF(IP.EQ.IPACTT) THEN
         CACC = '>'
         IF(LPFILE) THEN
          CFIL = '>'
         ELSE
          CFIL = ' '
         ENDIF
        ELSE
         CACC = ' '
         CFIL = ' '
        ENDIF
C
        IRET = IRETYP(IP)
        IMAT = IMATYP(IP)
C
        IF(REYNP1(IP).GT.0.0) THEN
         IEXP = INT( LOG10(REYNP1(IP)) )
         IEXP = MAX( MIN( IEXP , 9 ) , 0 )
         RMAN = REYNP1(IP) / 10.0**IEXP
        ELSE
         RMAN = 0.0
        ENDIF
C
        CALL STRIP(PFNAME(IP),NPF)
        WRITE(*,1200) CACC, IP, NAMEPOL(IP), 
     &                RMAN, IEXP, CLTYP(IRET), MACHP1(IP), CLTYP(IMAT),
     &                ACRITP(1,IP), ACRITP(2,IP),
     &                XSTRIPP(1,IP), XSTRIPP(2,IP),
     &                CFIL,PFNAME(IP)(1:NPF)
 1200   FORMAT(1X,A1,I3,2X, A24, F7.3,'e',I1,A5, F7.3,A5, 
     &            2F8.2, 2F8.3, 2X, A1, 1X, A)
      ENDDO
C
      RETURN
      END ! PLRSUM



      SUBROUTINE PRFSUM(IR1,IR2)
C---------------------------------------------
C     Prints summary of reference polars IR1..IR2
C---------------------------------------------
      INCLUDE 'XFOIL.INC'
C
 1100 FORMAT(1X,A,A)
      WRITE(*,*)
      WRITE(*,1100) '       reference polar                          '
      WRITE(*,1100) '      ------------------------------------------'
CCC                  123456789012345678901234567890123456789012345678
C
      DO IR = IR1, IR2
        WRITE(*,1200) IR, NAMEREF(IR)
 1200   FORMAT(1X,1X,I3,2X, A48)
      ENDDO
C
      RETURN
      END ! PRFSUM



      SUBROUTINE PLRCOP(IP1,IP2)
C---------------------------------------------
C     Copies polar in slot IP1 into slot IP2
C---------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      NAMEPOL(IP2) = NAMEPOL(IP1)
      CODEPOL(IP2) = CODEPOL(IP1)
      VERSPOL(IP2) = VERSPOL(IP1)
      PFNAME(IP2) = PFNAME(IP1)
      PFNAMX(IP2) = PFNAMX(IP1)
C
      MACHP1(IP2) = MACHP1(IP1)
      REYNP1(IP2) = REYNP1(IP1)
C
      IMATYP(IP2) = IMATYP(IP1)
      IRETYP(IP2) = IRETYP(IP1)

      ACRITP(1,IP2) = ACRITP(1,IP1)
      ACRITP(2,IP2) = ACRITP(2,IP1)
C
      XSTRIPP(1,IP2) = XSTRIPP(1,IP1)
      XSTRIPP(2,IP2) = XSTRIPP(2,IP1)
C
      NAPOL(IP2) = NAPOL(IP1)
      DO IA=1, NAPOL(IP2)
        DO ID = 1, IPTOT
          CPOL(IA,ID,IP2) = CPOL(IA,ID,IP1)
        ENDDO
        DO ID = 1, JPTOT
          CPOLSD(IA,1,ID,IP2) = CPOLSD(IA,1,ID,IP1)
          CPOLSD(IA,2,ID,IP2) = CPOLSD(IA,2,ID,IP1)
        ENDDO
      ENDDO
C
      NXYPOL(IP2) = NXYPOL(IP1)
      DO I = 1, NXYPOL(IP1)
        CPOLXY(I,1,IP2) = CPOLXY(I,1,IP1)
        CPOLXY(I,2,IP2) = CPOLXY(I,2,IP1)
      ENDDO
C
      RETURN
      END ! PLRCOP




      SUBROUTINE PRFCOP(IR1,IR2)
C---------------------------------------------
C     Copies reference polar in slot IR1 into slot IR2
C---------------------------------------------
      INCLUDE 'XFOIL.INC'
C
      NAMEREF(IR2) = NAMEREF(IR1)
C
      DO K = 1, 4
        NDREF(K,IR2) = NDREF(K,IR1)
      ENDDO
C
      DO IS = 1, 2
        DO K = 1, 4
          DO IA=1, NDREF(K,IR2)
            CPOLREF(IA,IS,K,IR2) = CPOLREF(IA,IS,K,IR1)
          ENDDO
        ENDDO
      ENDDO
C
      RETURN
      END ! PRFCOP


      SUBROUTINE POLAXI(CPOLPLF,XCDWID,XALWID,XOCWID)
C-------------------------------------------
C     Gets polar plot axis limits from user
C-------------------------------------------
      INCLUDE 'PINDEX.INC'
      DIMENSION CPOLPLF(3,*)
C
      LOGICAL ERROR
      CHARACTER*5 CVAR(4)
      DATA CVAR / 'Alpha' , '  CL ', '  CD ', ' -CM ' /
C
      WRITE(*,*) 'Enter new axis annotations,',
     &           ' or <return> to leave unchanged...'
      WRITE(*,*)
C
      DO KV=1, 4
 5      WRITE(*,1200) CVAR(KV), (CPOLPLF(J,KV), J=1, 3)
 1200   FORMAT(3X,A,'  min, max, delta:', 3F11.5)
        CALL READR(3,CPOLPLF(1,KV),ERROR)
        IF(ERROR) THEN
         WRITE(*,*) 'READ error.  Enter again.'
         GO TO 5
        ENDIF
      ENDDO
C
cC---- widths of plot boxes in polar plot page
c      XCDWID = 0.45
c      XALWID = 0.25
c      XOCWID = 0.20
C
      RETURN
      END ! POLAXI



      SUBROUTINE BOTTOM(LU)
      CHARACTER*1 DUMMY
C
 10   READ(LU,1000,END=90,ERR=90) DUMMY
 1000 FORMAT(A)
      GO TO 10
C
 90   RETURN
      END


      SUBROUTINE BOTTOMX(LU)
      CHARACTER*1 DUMMY
C
 10   READ(LU,END=90,ERR=90) DUMMY
      GO TO 10
C
 90   RETURN
      END


