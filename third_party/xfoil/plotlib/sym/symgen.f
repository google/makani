C*********************************************************************** 
C    Module:  symgen.f  (part of font generation tools in Xplot/sym)
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
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
C
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C*********************************************************************** 


      PROGRAM SYMGEN
C------------------------------------------------------------------------
C
C     Interactive program for creating and modifying vector fonts.
C
C
C     A font set is kept stored in a database file XXXX.FNT, which
C     is read in when the program is invoked with
C
C      % symgen XXXX
C
C     The first line of XXXX.FNT is a list of the defined characters.
C     Each subsequent line then defines one character in the list:
C
C  ABCD...
C  21616 12168 13280 ...      <- definition for "A"
C  21616 12280 14880 ...      <- definition for "B"
C   .
C   .
C
C     Each 5-digit number defines a vector node x,y location on a 96x96 
C     grid, with 16,16 being the lower-left "origin" of the character.
C     See SUBR. SYMPLT header below for the encoding format.
C     If the numbers are zero or missing, then the font character 
C     is not plotted (blank).
C
C     Once the database has been edited, command "W" will generate
C     a Fortran INCLUDE file XXXX.INC, with the vector font information
C     placed in DATA statements.  This is for use in a SYMBOL-type 
C     routine to allow plotting of the font.  Four sample databases,
C     include files, and routines already exist:
C
C       CHAR.FNT    CHAR.INC    SUBR. PLCHAR    (Upper,lower case letters)
C       SLAN.FNT    SLAN.INC    SUBR. PLSLAN    (Slanted version of CHAR )
C       MATH.FNT    MATH.INC    SUBR. PLMATH    (Latex-like Greek symbols)
C       SYMB.FNT    SYMB.INC    SUBR. PLSYMB    (Versatec plotting symbols)
C
C             also available:   SUBR. PLNUMB    (Whole floating-point numbers)
C
C     SUBR. PLCHAR, PLSLAN, and PLMATH (in ../plt_font.f) are identical 
C     except for their INCLUDE statements.  One can generate analogous 
C     files and plot routines for any custom font, e.g.
C
C       USER.FNT    USER.INC    SUBR. PLUSER
C
C
C     SUBR. PLSYMB is nearly the same, except that it takes an integer
C     argument ISYM = 0,1... to select the symbol defined on line ISYM+1:
C
C  ABCD...
C  21616 12168 13280 ...      <- definition for ISYM = 0
C  21616 12280 14880 ...      <- definition for ISYM = 1
C   .
C   .
C     The ASCII characters on the first line are ignored in PLSYMB.
C
C     PLSYMB also shifts the symbol down and left by 1/2 the symbol
C     size, so the symbol is centered on 0,0.  This duplicates the 
C     Versatec SYMBOL routine.  Note that PLSYMB can only plot one
C     symbol at a time.
C
C
C     WARNING:  The CHAR font implemented in PLCHAR is assumed to be
C               a "WYSIWIG" font, in which each font character closely
C               represents its ASCII index character.  It is used by
C               other libPlt routines (and also this program!), and
C               hence should not be significantly modified.
C
C
C------------------------------------------------------------------------
C
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
      INTEGER NODET(20)
C
      LOGICAL SAVED
C
      CHARACTER*80 ARGP1, STRING, RANGE
C
      CHARACTER*1 COPT, ANS, ALPH, ALPHO, ALPHT, KCHAR
      LOGICAL LABORT, LDONE
C
      SIZE = 6.0
      IDEV = 1
      IPSLU = 0
C
      CH = 0.7
      X0 = 0.0
      Y0 = 0.0
C
      NCHARS = 0
C
C---- get the XXXX working set name
      CALL GETARG(1,ARGP1)
      CALL LOAD(ARGP1)
C
C---- initial overlay character, plot-test string
      ALPHO = ' '
      STRING = ' '
C
C---- initial target character is first one in database
      ALPH = CHARS(1:1)
      SAVED = .TRUE.
C
      CALL PLINITIALIZE
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
C
      CALL GETCOLOR(ICOL0)
C
 1000 FORMAT(A)
C
 900  CONTINUE
      WRITE(*,1005)
 1005 FORMAT(/'   C hange target character'
     &       /'   I nput  target character vectors'
     &       /'   M odify target character vectors'
     &       /'   T ranslate target character'
     &       /'   O verlay other character'
     &       /'   L ean character(s)'
     &       /'   P lot specified character string'
     &       /'   A dd a new character to database'
     &       /'   D elete a character from database'
     &       /'   S ave database'
     &       /'   W rite include file from current database'
     &       /'   Z oom'
     &       /'   U nzoom'
     &       /'   Q uit')
C
 910  WRITE(*,1015) ALPH
 1015 FORMAT(/' Select option ( target character = ',A1,' ):  ', $)
      READ(*,1000) COPT
C
      IF(INDEX('Qq',COPT).NE.0) THEN
        IF(.NOT.SAVED) THEN
          WRITE(*,*)
          WRITE(*,*) 'Database not saved.  Really quit ?  N'
          READ(*,1000) ANS
          IF(INDEX('Yy',ANS) .EQ. 0) GO TO 900
        ENDIF
        CALL PLCLOSE
        STOP
      ENDIF
C
      NOPT = INDEX('CIMTOLPADSWZU',COPT) + INDEX('cimtolpadswzu',COPT)
C
      GOTO (10,20,30,40,50,60,70,80,90,100,110,120,130) NOPT
      GOTO 900
C
C===========================================
C---- select character
 10   CONTINUE
      WRITE(*,1010) CHARS(1:NCHAR)
 1010 FORMAT(/1X,A,
     &      //1X,'Select new target character: ',$)
      READ(*,1000) ALPH
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- input character
 20   CONTINUE
C
      KC = INDEX(CHARS,ALPH)
      IF(KC.EQ.0) THEN
        WRITE(*,*) 'Character is not in database.'
        GO TO 900
      ENDIF
C
C---- clear node-input buffer array
      DO 201 K=1, 20
        NODET(K) = 0
 201  CONTINUE
C
C
C---- small node-select symbol size
      CHS = 0.010*CH
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PABORT
      CALL PDONE
C
ccc      CALL PLCHAR(X0,Y0,CH,ALPH,0.0,1)
C
      CALL PLFLUSH
C
      WRITE(*,*)
      WRITE(*,*) 'Input stroke points (double-click to end a stroke)...'
C
      ILAST = -999
      JLAST = -999
      ISTAT = 2
C
      DO 205 K=1, 20
 204    CALL GETCURSORXY(XC,YC,KCHAR)
C
C------ exit if clicked inside "ABORT" box
        IF(LABORT(XC,YC)) THEN
         WRITE(*,*) 'Database unchanged.'
         GO TO 209
        ENDIF
C
C------ finish up if clicked inside "DONE" box
        IF(LDONE(XC,YC)) GO TO 206
C
C------ set integer coordinates and clip to within field
        I = INT(64.0*(XC-X0)/CH + 16.5)
        J = INT(64.0*(YC-Y0)/CH + 16.5)
        I = MAX( 0 , MIN(I,96) )
        J = MAX( 0 , MIN(J,96) )
C
        IF(ILAST.EQ.I .AND. JLAST.EQ.J) THEN
          ISTAT = 2
          ILAST = -999
          JLAST = -999
          WRITE(*,*) 'Will start new stroke...'
          GO TO 204
        ENDIF
C
        WRITE(*,1026) K, I, J
 1026   FORMAT(1X,'   node',I3,':   ', 2I5)
C
C------ set new node location in buffer array
        NODET(K) = 10000*ISTAT + 100*I + J
C
C------ put small symbol over actual node location
        XNODE = CH*FLOAT(I-16)/64.0
        YNODE = CH*FLOAT(J-16)/64.0
        IF(ILAST.EQ.-999 .OR. JLAST.EQ.-999) THEN
          CALL PLOT(XNODE,YNODE,3)
        ELSE
          CALL PLOT(XNODE,YNODE,2)
        ENDIF
        CALL NEWCOLORNAME('orange')
        CALL PLSYMB(XNODE,YNODE,CHS,1,0.0,0)
        CALL NEWCOLOR(ICOL0)
        CALL PLFLUSH
C
C------ set up for next input point
        ILAST = I
        JLAST = J
        ISTAT = 1
C
 205  CONTINUE
      WRITE(*,*) '20-node array limit reached'
 206  CONTINUE
C
C---- put buffer array into database
      KC = INDEX(CHARS,ALPH)
      DO 207 K=1, 20
        NODE(K,KC) = NODET(K)
 207  CONTINUE
      WRITE(*,*) 
      WRITE(*,*) 'Database updated.'
      SAVED = .FALSE.
C
C---- replot whole character
 209  CALL PLEND
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
C
      CALL PLEND
      GO TO 900
C
C===========================================
C---- modify character
 30   CONTINUE
      WRITE(*,1030) ALPH
 1030 FORMAT(/1X, 'Modifying character: ', A)
C
      KC = INDEX(CHARS,ALPH)
      IF(KC.EQ.0) THEN
        WRITE(*,*) 'Character is not in database.'
        GO TO 900
      ENDIF
C
C---- save database node array for restoration after abort
      DO 301 K=1, 20
        NODET(K) = NODE(K,KC)
 301  CONTINUE
C
      CALL PLTINI
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
C
ccc      CALL PLSYMB(X0,Y0,CH,ALPH,0.0,1)
c
C
C---- small node-select symbol size
      CHS = 0.010*CH
C
      CALL PABORT
      CALL PDONE
      CALL PLFLUSH
C
      WRITE(*,*)
      WRITE(*,*) 'Click on old/new point pairs...'
      DO 305 IPASS=1, 12345
C
        CALL GETCURSORXY(XC,YC,KCHAR)
C
C------ restore and exit if clicked inside "ABORT" box
        IF(LABORT(XC,YC)) THEN
         DO 3052 K=1, 20
           NODE(K,KC) = NODET(K)
 3052    CONTINUE
         WRITE(*,*) 'Database unchanged.'
         GO TO 309
        ENDIF
C
C------ finish up if clicked inside "DONE" box
        IF(LDONE(XC,YC)) GO TO 306
C
C------ set integer coordinates and clip to within field
        I = INT(64.0*(XC-X0)/CH + 16.5)
        J = INT(64.0*(YC-Y0)/CH + 16.5)
        I = MAX( 0 , MIN(I,96) )
        J = MAX( 0 , MIN(J,96) )
C
C------ find nearest vector node
        IDMIN = 1000000
        KMIN = 0
        DO 3054 K=1, 20
          ISTAT = NODE(K,KC) / 10000
          IF(ISTAT.EQ.0) GO TO 3055
C
          NODEB = NODE(K,KC) - ISTAT*10000
          IT = NODEB / 100
          JT = NODEB - 100*IT
C
          IDIST = (I - IT)**2 + (J - JT)**2
          IF(IDIST.LT.IDMIN) THEN
            IDMIN = IDIST
            KMIN = K
          ENDIF
 3054   CONTINUE
 3055   CONTINUE
C
        KT = KMIN
        IF(KT.EQ.0) THEN
          WRITE(*,*) 'Nearest point not found.'
          GO TO 900
        ENDIF
C
C------ plot small symbol on nearest vector node to identify it
        ISTAT = NODE(KT,KC) / 10000
        NODEB = NODE(KT,KC) - ISTAT*10000
        IT = NODEB / 100
        JT = NODEB - 100*IT
C
        CALL NEWCOLORNAME('red')
        XNODE = CH*FLOAT(IT-16)/64.0
        YNODE = CH*FLOAT(JT-16)/64.0
        CALL PLSYMB(XNODE,YNODE,CHS,1,0.0,0)
        FK = FLOAT(KT)
        CALL PLNUMB(XNODE+1.5*CHS,YNODE+1.5*CHS,3.0*CHS,FK,0.0,-1)
        CALL NEWCOLOR(ICOL0)
C
C------ now get new location for vector node KT
        CALL GETCURSORXY(XC,YC,KCHAR)
C
        IF(LABORT(XC,YC)) THEN
         DO 3056 K=1, 20
           NODE(K,KC) = NODET(K)
 3056    CONTINUE
         WRITE(*,*) 'No changes made.'
         GO TO 309
        ENDIF
C
        IF(LDONE(XC,YC)) GO TO 306
C
C------ integer coordinates for new location
        I = INT(64.0*(XC-X0)/CH + 16.5)
        J = INT(64.0*(YC-Y0)/CH + 16.5)
        I = MAX( 0 , MIN(I,96) )
        J = MAX( 0 , MIN(J,96) )
C
C------ encode coordinates into database
        NODE(KT,KC) = 10000*ISTAT + 100*I + J
C
C------ replot everything
        CALL PLEND
        CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
        CALL PABORT
        CALL PDONE
        CALL PLFLUSH
C
 305  CONTINUE
 306  CONTINUE
C
      WRITE(*,*) 
      WRITE(*,*) 'Database updated.'
      SAVED = .FALSE.
C
C---- replot final state
 309  CALL PLEND
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
C
      CALL PLEND
      GO TO 900
C
C===========================================
C---- translate current character
 40   CONTINUE
C
      DO K=1, 20
        NODET(K) = 0
      ENDDO
C
 41   WRITE(*,1040)
 1040 FORMAT(/' Enter dX, dY (in points):  ', $)
      READ(*,*,ERR=41) ID, JD
C
      IC = INDEX(CHARS,ALPH)
      DO 42 K=1, 20
C
C------ unpack node coordinates from database
        ISTAT = NODE(K,IC) / 10000
        IF(ISTAT.EQ.0) GO TO 43
C
        NODEB = NODE(K,IC) - ISTAT*10000
        I = NODEB / 100
        J = NODEB - 100*I
C
C------ shift coordinates
        I = I + ID
        J = J + JD
C
        IF(I.LT.0 .OR. I.GT.96) THEN
          WRITE(*,*) 'X movement puts character outside field 0..96'
          GO TO 40
        ENDIF
C
        IF(J.LT.0 .OR. J.GT.96) THEN
          WRITE(*,*) 'Y movement puts character outside field 0..96'
          GO TO 40
        ENDIF
C
C------ encode new coordinates in buffer array
        NODET(K) = 10000*ISTAT + 100*I + J
C
 42   CONTINUE
 43   CONTINUE
C
C---- store shifted nodes in database from buffer array
      DO 46 K=1, 20
        NODE(K,IC) = NODET(K)
 46   CONTINUE
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- select overlay character
 50   CONTINUE
      ALPHO = ' '
      WRITE(*,1050) CHARS(1:NCHAR)
 1050 FORMAT(/1X,A,
     &      //1X,'Select character to overlay (<return> if none: ',$)
      READ(*,1000) ALPHO
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- lean character(s)
 60   CONTINUE
      WRITE(*,1060)
 1060 FORMAT(/' Enter character range to lean (e.g. AZ): ',$)
      READ(*,1000) RANGE
      IF(INDEX(RANGE,' ').EQ.1) GO TO 900
C
 62   WRITE(*,1062)
 1062 FORMAT(/1X,'Enter tan(lean_angle), + to right:  ',$)
      READ (*,*,ERR=62) TANA
C
      CALL LEAN(RANGE,TANA)
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- enter plot test string
 70   CONTINUE
      WRITE(*,1070)
 1070 FORMAT(/' Enter string: ',$)
      READ(*,1000) STRING
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- add character to database
 80   CONTINUE
      IF(NCHAR+1.GE.100) THEN
        WRITE(*,*) 
        WRITE(*,*) 'Array limit reached.  Cannot add character.'
        GO TO 900
      ENDIF
C
      WRITE(*,1080) 
 1080 FORMAT(/1X,'Enter character to be added: ',$)
      READ(*,1000) ALPH
      IC = INDEX(CHARS,ALPH)
      IF(IC.NE.0) THEN
        WRITE(*,*) 'That is already in database.'
        GO TO 900
      ENDIF
C
 81   WRITE(*,1081) CHARS(1:NCHAR), ALPH
 1081 FORMAT(/1X,A,
     &      //1X,' ... insert ',A1,
     &       ' before which character ? (<return> to append) : ',$)
      READ(*,1000) ALPHT
      IC = INDEX(CHARS,ALPHT)
      IF(IC.EQ.0) IC = NCHAR+1
C
C---- move up all characters above IC to make room for new character
      IF(IC.LE.NCHAR) THEN
        CHARS(IC+1:NCHAR+1) = CHARS(IC:NCHAR)
        DO 802 KC=NCHAR, IC, -1
          DO 8024 K=1, 20
            NODE(K,KC+1) = NODE(K,KC)
 8024     CONTINUE
 802    CONTINUE
      ENDIF
C
      NCHAR = NCHAR+1
C
C---- set new character and clear its vectors
      CHARS(IC:IC) = ALPH
      DO 804 K=1, 20
        NODE(K,IC) = 0
 804  CONTINUE
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- delete character from database
 90   CONTINUE
C
      WRITE(*,1090) 
 1090 FORMAT(/1X,'Select character to be deleted: ',$)
      READ(*,1000) ALPH
C
      IF(INDEX(' ',ALPH).EQ.1) THEN
       WRITE(*,*) 'No action taken'
       GO TO 900
      ENDIF
C
      IC = INDEX(CHARS,ALPH)
C
      IF(IC.EQ.0) THEN
       WRITE(*,*) 'Character not in database.'
       GO TO 900
      ENDIF
C
C---- pull down all characters above the one to be deleted
      CHARS(IC:NCHAR) = CHARS(IC+1:NCHAR+1)
      DO 902 KC=IC, NCHAR-1
        DO 9024 K=1, 20
          NODE(K,KC) = NODE(K,KC+1)
 9024   CONTINUE
 902  CONTINUE
      NCHAR = NCHAR-1
      IC = MIN(IC,NCHAR)
C
C---- set new current character
      ALPH = CHARS(IC:IC)
C
      CALL PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      CALL PLFLUSH
      CALL PLEND
      GO TO 900
C
C===========================================
C---- write out database
 100  CONTINUE
      CALL SAVE(ARGP1,SAVED)
      GO TO 900
C
C===========================================
C---- write out include file
 110  CONTINUE
      CALL WRIT(ARGP1)
      GO TO 910
C
C===========================================
C---- set zoom
 120  CONTINUE
      CALL USETZOOM(.FALSE.,.TRUE.)
      CALL REPLOT(1)
      GO TO 910
C
C===========================================
C---- clear zoom
 130  CONTINUE
      CALL CLRZOOM
      CALL REPLOT(1)
      GO TO 910
C
      END



      SUBROUTINE LOAD(ARGP1)
      CHARACTER*80 ARGP1,FNAME
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
 1000 FORMAT(A)
C
      K = INDEX(ARGP1,' ') - 1
      IF(K.LT.1) THEN
       WRITE(*,*) 'SYMGEN argument error: ', ARGP1
       STOP
      ENDIF
C
      FNAME = ARGP1(1:K) // '.FNT'
C
      OPEN(3,FILE=FNAME,STATUS='OLD',ERR=90)
      READ(3,1000) CHARS
      NCHAR = INDEX(CHARS,' ') - 1
C
      DO IC=1, NCHAR
        DO K=1, 20
          NODE(K,IC) = 0
        ENDDO
      ENDDO
C
      DO IC=1, NCHAR
        READ(3,*,END=80) (NODE(K,IC),K=1,20)
C
C------ clear any invalid points (should be zero anyway)
        DO K=1, 20
          ISTAT = NODE(K,IC) / 10000
          IF(ISTAT.EQ.0) NODE(K,IC) = 0
        ENDDO
      ENDDO
C
 80   CONTINUE
      CLOSE(3)
C
      WRITE(*,1200) CHARS(1:NCHAR)
 1200 FORMAT(/1X,'Database read for the following character set...'
     &      //1X, A)
C
      RETURN
C
 90   CONTINUE
      WRITE(*,*) 'Database file not found:  ',FNAME(1:60)
      RETURN
C
      END



      SUBROUTINE SAVE(ARGP1,SAVED)
      CHARACTER*80 ARGP1,FNAME
      CHARACTER*1 ANS
      CHARACTER*4 CNUM
      LOGICAL SAVED
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
C
 1000 FORMAT(A)
 1500 FORMAT(1X,20I6)
 2100 FORMAT(/' File ',A,' exits.  Overwrite ?  Y')
C
C
      KARG = INDEX(ARGP1,' ') - 1
C
C---- write new database file for next SYMGEN call
      FNAME = ARGP1(1:KARG) // '.FNT'
      OPEN(3,FILE=FNAME,STATUS='OLD',ERR=10)
C
      WRITE(*,2100) FNAME(1:KARG+4)
      READ(*,1000) ANS
      IF(INDEX('Nn',ANS) .NE. 0) THEN
        CLOSE(3)
        RETURN
      ENDIF
C
 10   OPEN(3,FILE=FNAME,STATUS='UNKNOWN')
C
      WRITE(3,1000) CHARS(1:NCHAR)
      DO IC=1, NCHAR
        WRITE(3,1500) (NODE(K,IC),K=1,20)
      ENDDO
      CLOSE(3)
      WRITE(*,*)
      WRITE(*,*) 'New database file written: ', FNAME(1:KARG+4)
      SAVED = .TRUE.
C
      RETURN
      END



      SUBROUTINE WRIT(ARGP1)
      CHARACTER*80 ARGP1,FNAME,LINE
      CHARACTER*1 ANS
      CHARACTER*4 CNUM
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
C---- write new include file for SYMBOL-type routine
C
 1000 FORMAT(A)
 1500 FORMAT(1X,20I6)
 2100 FORMAT(/' File ',A,' exits.  Overwrite ?  Y')
C
      KARG = INDEX(ARGP1,' ') - 1
C
      FNAME = ARGP1(1:KARG) // '.INC'
      OPEN(4,FILE=FNAME,STATUS='OLD',ERR=20)
C
      WRITE(*,2100) FNAME(1:KARG+4)
      READ(*,1000) ANS
      IF(INDEX('Nn',ANS) .NE. 0) THEN
        CLOSE(4)
        GO TO 28
      ENDIF
C
 20   OPEN(4,FILE=FNAME,STATUS='UNKNOWN')
      WRITE(4,*) ' '
      WRITE(4,3010) NCHAR, NCHAR
C
      WRITE(4,*) ' '
      NARR = (NCHAR-1)/10
      DO N=0, NARR
        NDIM = MIN( 10 , NCHAR-N*10)
        WRITE(4,3201) N, NDIM
      ENDDO
      DO N=0, NARR
        WRITE(4,3202) 10*N+1, N
      ENDDO
C
C---Write character translation data statements
      WRITE(4,3020) NCHAR
      DO L=1, 8
        K1 = 26*L - 25
        K2 = 26*L
        IF(K1 .GT. NCHAR) GO TO 21
        K2 = MIN(K2,NCHAR)
        WRITE(4,3021) K1,K2, CHARS(K1:K2)
      ENDDO
 21   CONTINUE
C
C---Write out character node data
      DO N=0, NARR
        IC1 = 10*N + 1
        IC2 = 10*N + 10
        IC2 = MIN(IC2,NCHAR)
C
        IC = IC1
        WRITE(4,3210) N
        DO IC=IC1, IC2-1
          WRITE(4,3220) (NODE(K,IC),K= 1,10)
          WRITE(4,3220) (NODE(K,IC),K=11,20)
        ENDDO
        IC = IC2
        WRITE(4,3220) (NODE(K,IC),K= 1,10)
        WRITE(4,3230) (NODE(K,IC),K=11,20)
      ENDDO
C
      CLOSE(4)
C
 3010 FORMAT('      CHARACTER*',I3,' CHARS'
     &      /'      INTEGER NODE(20,',I3,')')
C
 3201 FORMAT('      DIMENSION NODE',I1,'(20,',I2,')')
 3202 FORMAT('      EQUIVALENCE ( NODE(1,',I3,') , NODE',I1,'(1,1) )')
C
 3020 FORMAT('      DATA NCHARS / ',I3,' /')
 3021 FORMAT('      DATA CHARS(',I3,':',I3,') / ''',A,''' /')
C
 3210 FORMAT( '      DATA NODE',I1,' /')
 3220 FORMAT( '     & ',10(I5,',') )
 3230 FORMAT( '     & ', 9(I5,','),I5,' /' )
C
      CLOSE(4)
      WRITE(*,*)
      WRITE(*,*) 'New include file written: ', FNAME(1:KARG+4)
      WRITE(*,*) 'Put include file in Xplot directory,',
     &          '  make libPlt.a  to implement new font.'
C
 28   CONTINUE
      RETURN
C
      END


      SUBROUTINE LEAN(RANGE,TANA)
      CHARACTER*(*) RANGE
C
      CHARACTER*1 ALPH
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
      KC1 = INDEX(CHARS,RANGE(1:1))
      KC2 = INDEX(CHARS,RANGE(2:2))
C
      IF(KC1.EQ.0 .OR. KC2.EQ.0) THEN
       WRITE(*,*) 'Specified range not in current character set.'
       WRITE(*,*) 'No action taken.'
       RETURN
      ENDIF
C
C---- go over each character...
      DO 12 KC=KC1, KC2
C
C------ go over each node
        DO K=1, 20
C
C-------- strip off leading point-status digit
          ISTAT = NODE(K,KC) / 10000
          NODEB = NODE(K,KC) - ISTAT*10000
C
          IF(ISTAT.GT.0) THEN
C
C---------- decode x and y location coordinates
            I = NODEB / 100
            J = NODEB - 100*I
C
C---------- perform tilt
            I = I + INT( FLOAT(J-16)*TANA )
C
C---------- encode coordinates back into database
            NODE(K,KC) = 10000*ISTAT + 100*I + J
C
          ENDIF
C
        ENDDO
C
 12   CONTINUE
C
      RETURN
      END



      SUBROUTINE PLTINI
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
C
      RELWSIZ = 0.8
      CALL PLOPEN(RELWSIZ,IPSLU,IDEV)
      CALL PLOTABS(0.125,0.125,-3)
      CALL NEWFACTOR(SIZE)
      CALL PLOT(0.25*CH,0.25*CH,-3)
C
      RETURN
      END ! PLTINI



      SUBROUTINE PLTALL(X0,Y0,ALPH,ALPHO,STRING)
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
      CHARACTER*1 ALPH, ALPHO
      CHARACTER*80 STRING
C
      CHSEQ = 0.10*CH
      CHSTR = 0.05*CH
C
      XSEQ = CH + CH/4.0 + 0.03*CH
      YSEQ = CH + CH/4.0 - 1.10*CHSEQ
C
      XCHR = CH + CH/4.0 + 0.03*CH
      YCHR = CH          - 0.07*CH
C
      XSTR = XCHR
      YSTR = YCHR - 0.65*CH
C
      CALL PLTINI
      CALL PLTGRD(X0,Y0,CH)
      CALL SYMPLT(X0,Y0,CH,ALPH,0.0,-1)
      CALL SYMPLT(X0,Y0,CH,ALPHO,0.0,1)
      CALL SYMPLT(XSTR,YSTR,CHSTR,STRING,0.0,80)
      CALL PLTCHR(XCHR,YCHR)
C
      XQ  = XSEQ
      CHQ = CHSEQ
c      CALL PLOT(XQ,YSEQ,3)
      DO ISCAL=0, 10
        CALL SYMPLT(XQ,YSEQ,CHQ,ALPH,0.0,1)
c        CALL PLCHAR(999.,999.,CHQ,ALPH,0.0,1)
        XQ  = XQ + CHQ
        CHQ = CHQ * 0.75
      ENDDO
C
      RETURN
      END



      SUBROUTINE PLTCHR(XCHR,YCHR)
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
C
      CHARACTER*1 ALPH
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
      CHP = 0.040*CH
C
      DO 10 ILIN=1, 5
        YPLT = YCHR - CHP - 2.5*CHP*(ILIN-1)
        DO 102 KC=1, 26
          XPLT = XCHR + 0.5*CHP + 1.0*CHP*(KC-1)
          XPLT = XCHR + 1.0*CHP*(KC-1)
C
          IC = KC + 26*(ILIN-1)
          IF(IC.GT.NCHAR) GO TO 15
C
          ALPH = CHARS(IC:IC)
          CALL SYMPLT(XPLT,YPLT,CHP,ALPH,0.0,1)
 102    CONTINUE
 10   CONTINUE
C
 15   CONTINUE
      RETURN
      END



      SUBROUTINE SYMPLT(XC,YC,CH,STRING,ANGLE,NC1)
C----------------------------------------------------------------
C     Plots character string using vector font database NODE(..)
C
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
      CHARACTER*1 ALPH
C
      CHARACTER*100 CHARS
      COMMON /CDATA/ CHARS
      COMMON /NDATA/ NCHAR, NODE(20,100)
C
      NC = ABS(NC1)
      CHS = 0.010*CH
C
      CALL GETCOLOR(ICOL0)
C
C---- go over each character...
      DO 12 IC=1, NC
C
C------ set plot location
        X0 = XC + CH*FLOAT(IC-1)
        Y0 = YC
C
        ALPH = STRING(IC:IC)
        KC = INDEX(CHARS,ALPH)
C
        IF(KC.NE.0) THEN
C
C-------- decode and plot each node
          DO K=1, 20
C
C---------- strip off leading point-status digit
            ISTAT = NODE(K,KC) / 10000
            NODEB = NODE(K,KC) - ISTAT*10000
C
C---------- decode x and y location coordinates
            I = NODEB / 100
            J = NODEB - 100*I
C
            X = X0 + CH*FLOAT(I-16)/64.0
            Y = Y0 + CH*FLOAT(J-16)/64.0
C
            IF     (ISTAT.EQ.0) THEN
             GOTO 12
            ELSEIF (ISTAT.EQ.1) THEN
             CALL PLOT(X,Y,2)
            ELSE
             CALL PLOT(X,Y,3)
            ENDIF
C
C---------- plot symbol at vector nodes
            IF(NC1.LT.0) THEN
              CALL NEWCOLORNAME('blue')
              CALL PLSYMB(X,Y,CHS,1,0.0,0)
              FK = FLOAT(K)
              CALL PLNUMB(X+1.5*CHS,Y+1.5*CHS,3.0*CHS,FK,0.0,-1)
              CALL NEWCOLOR(ICOL0)
              CALL PLOT(X,Y,3)
            ENDIF
C
          ENDDO
        ENDIF
C
 12   CONTINUE
C
      RETURN
      END



      SUBROUTINE PLTGRD(X0,Y0,CH)
      DATA LMASK1, LMASK2, LMASK3 / -32640, -30584, -21846 /
C
      CALL NEWPEN(1)
C
      NX = 24
      NY = 24
      DX = CH/16.0
      DY = CH/16.0
      CALL PLGRID(X0-0.25*CH,Y0-0.25*CH, NX,DX, NY,DY, LMASK2 )
C
      NX = 2
      NY = 2
      DX = CH/2.0
      DY = CH/2.0
      CALL PLGRID(X0,Y0, NX,DX, NY,DY, LMASK3 )
C
      DO I=0,2
        DO J=0,2
          XPLT = X0 + DX*FLOAT(I)
          YPLT = Y0 + DY*FLOAT(J)
          CALL PLSYMB(XPLT,YPLT,CH/96.0,5,0.0,0)
        ENDDO
      ENDDO
C
      CALL PLFLUSH
C
      RETURN
      END




      SUBROUTINE PDONE
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
      COMMON /DONE/ XDONE(2), YDONE(2)
C
C---- set DONE window
      XDONE(1) = CH + CH/4.0 + 0.125/SIZE
      XDONE(2) = CH + CH/4.0 + 1.025/SIZE
      YDONE(1) =    - CH/4.0 + 0.525/SIZE
      YDONE(2) =    - CH/4.0 + 1.425/SIZE
C
C---- plot DONE window
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('green')
      CALL PLOT(XDONE(1),YDONE(1),3)
      CALL PLOT(XDONE(2),YDONE(1),2)
      CALL PLOT(XDONE(2),YDONE(2),2)
      CALL PLOT(XDONE(1),YDONE(2),2)
      CALL PLOT(XDONE(1),YDONE(1),2)
C
      CHA = MIN( (XDONE(2)-XDONE(1))/8.0 , (YDONE(2)-YDONE(1))/1.5 )
      XCA = 0.5*(XDONE(2)+XDONE(1)) - 2.0*CHA
      YCA = 0.5*(YDONE(2)+YDONE(1)) - 0.5*CHA
      CALL PLCHAR(XCA,YCA,CHA,'DONE',0.0,4)
      CALL NEWCOLOR(ICOL0)
C
      RETURN
      END


      FUNCTION LDONE(XC,YC)
      COMMON /DONE/ XDONE(2), YDONE(2)
      LOGICAL LDONE
C
C---- return T if location XC,YC falls within DONE window
C
      LDONE = XC .GE. XDONE(1) .AND. 
     &        XC .LE. XDONE(2) .AND.
     &        YC .GE. YDONE(1) .AND.
     &        YC .LE. YDONE(2)
C
      RETURN
      END




      SUBROUTINE PABORT
      COMMON /PLTC/ IDEV, IPSLU, SIZE, CH
      COMMON /ABRT/ XABORT(2), YABORT(2)
C
C---- set abort window
      XABORT(1) =  CH + CH/4.0 + 0.125/SIZE
      XABORT(2) =  CH + CH/4.0 + 1.025/SIZE
      YABORT(1) =     - CH/4.0
      YABORT(2) =     - CH/4.0 + 0.400/SIZE
C
C---- plot abort window
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('red')
      CALL PLOT(XABORT(1),YABORT(1),3)
      CALL PLOT(XABORT(2),YABORT(1),2)
      CALL PLOT(XABORT(2),YABORT(2),2)
      CALL PLOT(XABORT(1),YABORT(2),2)
      CALL PLOT(XABORT(1),YABORT(1),2)
C
      CHA = MIN( (XABORT(2)-XABORT(1))/8.0 , (YABORT(2)-YABORT(1))/1.5 )
      XCA = 0.5*(XABORT(2)+XABORT(1)) - 2.5*CHA
      YCA = 0.5*(YABORT(2)+YABORT(1)) - 0.5*CHA
      CALL PLCHAR(XCA,YCA,CHA,'ABORT',0.0,5)
      CALL NEWCOLOR(ICOL0)
C
      RETURN
      END


      FUNCTION LABORT(XC,YC)
      COMMON /ABRT/ XABORT(2), YABORT(2)
      LOGICAL LABORT
C
C---- return T if location XC,YC falls within abort window
C
      LABORT = XC .GE. XABORT(1) .AND. 
     &         XC .LE. XABORT(2) .AND.
     &         YC .GE. YABORT(1) .AND.
     &         YC .LE. YABORT(2)
C
      RETURN
      END


