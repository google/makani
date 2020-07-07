C***********************************************************************
C    Module:  userio.f
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
C
C
C==== user input routines with prompting and error trapping
C
C
      SUBROUTINE ASKI(PROMPT,IINPUT)
C
C---- integer input
C
      CHARACTER*(*) PROMPT
      INTEGER IINPUT
      CHARACTER LINE*80
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
 10   WRITE(*,1000) PROMPT(1:NP)
C
      READ (*,1001,ERR=10) LINE
      IF(LINE.NE.' ') THEN
        READ (LINE,*,ERR=10) IINPUT
      ENDIF  
      RETURN
C
 1000 FORMAT(/A,'   i>  ',$)
 1001 FORMAT(A)
      END ! ASKI


      SUBROUTINE ASKR(PROMPT,RINPUT)
C
C---- real input
C
      CHARACTER*(*) PROMPT
      REAL RINPUT
      CHARACTER LINE*80
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
 10   WRITE(*,1000) PROMPT(1:NP)
C
      READ (*,1001,ERR=10) LINE
      IF(LINE.NE.' ') THEN
        READ (LINE,*,ERR=10) RINPUT
      ENDIF  
      RETURN
C
 1000 FORMAT(/A,'   r>  ',$)
 1001 FORMAT(A)
      END ! ASKR


      SUBROUTINE ASKL(PROMPT,LINPUT)
C
C---- logical input
C
      CHARACTER*(*) PROMPT
      LOGICAL LINPUT
      CHARACTER*1 CHAR
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
 10   WRITE(*,1000) PROMPT(1:NP)
      READ (*,1010) CHAR
      IF(CHAR.EQ.'y') CHAR = 'Y'
      IF(CHAR.EQ.'n') CHAR = 'N'
      IF(CHAR.NE.'Y' .AND. CHAR.NE.'N') GO TO 10
C
      LINPUT = CHAR .EQ. 'Y'
      RETURN
C
 1000 FORMAT(/A,' y/n>  ',$)
 1010 FORMAT(A)
      END ! ASKL


      SUBROUTINE ASKS(PROMPT,INPUT)
C
C---- string of arbitrary length input
C
      CHARACTER*(*) PROMPT
      CHARACTER*(*) INPUT
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
      WRITE(*,1000) PROMPT(1:NP)
      READ (*,1010) INPUT
C
      RETURN
C
 1000 FORMAT(/A,'   s>  ',$)
 1010 FORMAT(A)
      END ! ASKS


      SUBROUTINE ASKC(PROMPT,COMAND,CARGS)
C
C---- returns 4-byte character string input converted to uppercase
C---- also returns rest of input characters in CARGS string
C
      CHARACTER*(*) PROMPT
      CHARACTER*(*) COMAND, CARGS
C
      CHARACTER*128 LINE
      LOGICAL ERROR
C
      IZERO = ICHAR('0')
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
      WRITE(*,1000) PROMPT(1:NP)
      READ (*,1020) LINE
C
C---- strip off leading blanks
      DO K=1, 128
        IF(LINE(1:1) .EQ. ' ') THEN
         LINE = LINE(2:128)
        ELSE
         GO TO 5
        ENDIF
      ENDDO
 5    CONTINUE
C
C---- find position of first blank, "+", "-", ".", ",", or numeral
      K = INDEX(LINE,' ')
      KI = INDEX(LINE,'-')
      IF(KI.NE.0) K = MIN(K,KI)
      KI = INDEX(LINE,'+')
      IF(KI.NE.0) K = MIN(K,KI)
      KI = INDEX(LINE,'.')
      IF(KI.NE.0) K = MIN(K,KI)
      KI = INDEX(LINE,',')
      IF(KI.NE.0) K = MIN(K,KI)
      DO I=0, 9
        KI = INDEX(LINE,CHAR(IZERO+I))
        IF(KI.NE.0) K = MIN(K,KI)
      ENDDO
C
C---- there is no blank between command and argument... use first 4 characters
      IF(K.LE.0) K = 5
C
      IF(K.EQ.1) THEN
C------ the "command" is a number... set entire COMAND string with it
        COMAND = LINE
      ELSE
C------ the "command" is some string... just use the part up to the argument
        COMAND = LINE(1:K-1)
      ENDIF
C
C---- convert it to uppercase
      CALL LC2UC(COMAND)
C
      CARGS = LINE(K:128)
      CALL STRIP(CARGS,NCARGS)
      RETURN
C
 1000 FORMAT(/A,'   c>  ',$)
 1020 FORMAT(A)
      END ! ASKC


      SUBROUTINE LC2UC(INPUT)
      CHARACTER*(*) INPUT
C
      CHARACTER*26 LCASE, UCASE
      DATA LCASE / 'abcdefghijklmnopqrstuvwxyz' /
      DATA UCASE / 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' /
C
      N = LEN(INPUT)
C
      DO 10 I=1, N
        K = INDEX( LCASE , INPUT(I:I) )
        IF(K.GT.0) INPUT(I:I) = UCASE(K:K)
 10   CONTINUE
C
      RETURN
      END ! LC2UC



      SUBROUTINE READI(N,IVAR,ERROR)
      DIMENSION IVAR(N)
      LOGICAL ERROR
C--------------------------------------------------
C     Reads N integer variables, leaving unchanged 
C     if only <return> is entered.
C--------------------------------------------------
      DIMENSION IVTMP(40)
      CHARACTER*80 LINE
C
      READ(*,1000) LINE
 1000 FORMAT(A80)
C
      DO 10 I=1, N
        IVTMP(I) = IVAR(I)
 10   CONTINUE
C
      NTMP = 40
      CALL GETINT(LINE,IVTMP,NTMP,ERROR)
C
      IF(ERROR) RETURN
C
      DO 20 I=1, N
        IVAR(I) = IVTMP(I)
 20   CONTINUE
C
      RETURN
      END ! READI



      SUBROUTINE READR(N,VAR,ERROR)
      DIMENSION VAR(N)
      LOGICAL ERROR
C-------------------------------------------------
C     Reads N real variables, leaving unchanged 
C     if only <return> is entered.
C-------------------------------------------------
      DIMENSION VTMP(40)
      CHARACTER*80 LINE
C
      READ(*,1000) LINE
 1000 FORMAT(A80)
C
      DO 10 I=1, N
        VTMP(I) = VAR(I)
 10   CONTINUE
C
      NTMP = 40
      CALL GETFLT(LINE,VTMP,NTMP,ERROR)
C
      IF(ERROR) RETURN
C
      DO 20 I=1, N
        VAR(I) = VTMP(I)
 20   CONTINUE
C
      RETURN
      END ! READR




      SUBROUTINE GETINT(INPUT,A,N,ERROR)
      CHARACTER*(*) INPUT
      INTEGER A(*)
      LOGICAL ERROR
C----------------------------------------------------------
C     Parses character string INPUT into an array
C     of integer numbers returned in A(1...N)
C
C     Will attempt to extract no more than N numbers, 
C     unless N = 0, in which case all numbers present
C     in INPUT will be extracted.
C
C     N returns how many numbers were actually extracted.
C----------------------------------------------------------
      CHARACTER*130 REC
      CHARACTER*1 TAB
C
      TAB = CHAR(9)
C
C---- only first 128 characters in INPUT will be parsed
      ILEN = MIN( LEN(INPUT) , 128 )
      ILENP = ILEN + 2
C
C---- put input into local work string (which will be munched)
      REC(1:ILENP) = INPUT(1:ILEN) // ' ,'
C
C---- ignore everything after a "!" character
      K = INDEX(REC,'!')
      IF(K.GT.0) REC(1:ILEN) = REC(1:K-1)
C
C---- change tabs to spaces
 5    K = INDEX(REC(1:ILEN),TAB)
      IF(K.GT.0) THEN
       REC(K:K) = ' '
       GO TO 5
      ENDIF
C
      NINP = N
C
C---- count up how many numbers are to be extracted
      N = 0
      K = 1
      DO 10 IPASS=1, ILEN
C------ search for next space or comma starting with current index K
        KSPACE = INDEX(REC(K:ILENP),' ') + K - 1
        KCOMMA = INDEX(REC(K:ILENP),',') + K - 1
C
        IF(K.EQ.KSPACE) THEN
C------- just skip this space
         K = K+1
         GO TO 9
        ENDIF
C
        IF(K.EQ.KCOMMA) THEN
C------- comma found.. increment number count and keep looking
         N = N+1
         K = K+1
         GO TO 9
        ENDIF
C
C------ neither space nor comma found, so we ran into a number...
C-    ...increment number counter and keep looking after next space or comma
        N = N+1
        K = MIN(KSPACE,KCOMMA) + 1
C
  9     IF(K.GE.ILEN) GO TO 11
 10   CONTINUE
C
C---- decide on how many numbers to read, and go ahead and read them
 11   IF(NINP.GT.0) N = MIN( N, NINP )
      READ(REC(1:ILEN),*,ERR=20) (A(I),I=1,N)
      ERROR = .FALSE.
      RETURN
C
C---- bzzzt !!!
 20   CONTINUE
ccc   WRITE(*,*) 'GETINT: String-to-integer conversion error.'
      N = 0
      ERROR = .TRUE.
      RETURN
      END ! GETINT


      SUBROUTINE GETFLT(INPUT,A,N,ERROR)
      CHARACTER*(*) INPUT
      REAL A(*)
      LOGICAL ERROR
C----------------------------------------------------------
C     Parses character string INPUT into an array
C     of real numbers returned in A(1...N)
C
C     Will attempt to extract no more than N numbers, 
C     unless N = 0, in which case all numbers present
C     in INPUT will be extracted.
C
C     N returns how many numbers were actually extracted.
C----------------------------------------------------------
      CHARACTER*130 REC
      CHARACTER*1 TAB
C
      TAB = CHAR(9)
C
C---- only first 128 characters in INPUT will be parsed
      ILEN = MIN( LEN(INPUT) , 128 )
      ILENP = ILEN + 2
C
C---- put input into local work string (which will be munched)
      REC(1:ILENP) = INPUT(1:ILEN) // ' ,'
C
C---- ignore everything after a "!" character
      K = INDEX(REC,'!')
      IF(K.GT.0) REC(1:ILEN) = REC(1:K-1)
C
C---- change tabs to spaces
 5    K = INDEX(REC(1:ILEN),TAB)
      IF(K.GT.0) THEN
       REC(K:K) = ' '
       GO TO 5
      ENDIF
C
      NINP = N
C
C---- count up how many numbers are to be extracted
      N = 0
      K = 1
      DO 10 IPASS=1, ILEN
C------ search for next space or comma starting with current index K
        KSPACE = INDEX(REC(K:ILENP),' ') + K - 1
        KCOMMA = INDEX(REC(K:ILENP),',') + K - 1
C
        IF(K.EQ.KSPACE) THEN
C------- just skip this space
         K = K+1
         GO TO 9
        ENDIF
C
        IF(K.EQ.KCOMMA) THEN
C------- comma found.. increment number count and keep looking
         N = N+1
         K = K+1
         GO TO 9
        ENDIF
C
C------ neither space nor comma found, so we ran into a number...
C-    ...increment number counter and keep looking after next space or comma
        N = N+1
        K = MIN(KSPACE,KCOMMA) + 1
C
  9     IF(K.GE.ILEN) GO TO 11
 10   CONTINUE
C
C---- decide on how many numbers to read, and go ahead and read them
 11   IF(NINP.GT.0) N = MIN( N, NINP )
      READ(REC(1:ILEN),*,ERR=20) (A(I),I=1,N)
      ERROR = .FALSE.
      RETURN
C
C---- bzzzt !!!
 20   CONTINUE
ccc   WRITE(*,*) 'GETFLT: String-to-integer conversion error.'
      N = 0
      ERROR = .TRUE.
      RETURN
      END ! GETFLT



      SUBROUTINE STRIP(STRING,NS)
      CHARACTER*(*) STRING
C----------------------------------------------------
C     Strips leading blanks off STRING and returns 
C     length NS of non-blank part.
C----------------------------------------------------
      NLEN = LEN(STRING)
C
C---- find last non-blank character
      DO K2 = NLEN, 1, -1
        IF(STRING(K2:K2).NE.' ') GO TO 11
      ENDDO
      K2 = 0
   11 CONTINUE
C
C---- find first non-blank character
      DO K1 = 1, K2
        IF(STRING(K1:K1).NE.' ') GO TO 21
      ENDDO
   21 CONTINUE
C
C---- number of non-blank characters
      NS = K2 - K1 + 1
      IF(NS.EQ.0) RETURN
C
C---- shift STRING so first character is non-blank
      STRING(1:NS) = STRING(K1:K2)
C
C---- pad tail of STRING with blanks
      DO K = NS+1, NLEN
        STRING(K:K) = ' '
      ENDDO
C
      RETURN
      END







      SUBROUTINE GETARG0(IARG,ARG)
C------------------------------------------------
C     Same as GETARG, but...
C
C     ...in the case of Intel Fortran, this one
C     doesn't barf if there's no Unix argument 
C      (just returns blank string instead)
C------------------------------------------------
      CHARACTER*(*) ARG
C
      NARG = IARGC()
      IF(NARG.GE.IARG) THEN
       CALL GETARG(IARG,ARG)
      ELSE
       ARG = ' '
      ENDIF
C
      RETURN
      END ! GETARG0

