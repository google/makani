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
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
 10   WRITE(*,1000) PROMPT(1:NP)
      READ (*,*,ERR=10) IINPUT
      RETURN
C
 1000 FORMAT(/A,'   i>  ',$)
      END ! ASKI


      SUBROUTINE ASKR(PROMPT,RINPUT)
C
C---- real input
C
      CHARACTER*(*) PROMPT
      REAL RINPUT
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
 10   WRITE(*,1000) PROMPT(1:NP)
      READ (*,*,ERR=10) RINPUT
      RETURN
C
 1000 FORMAT(/A,'   r>  ',$)
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


      SUBROUTINE ASKC(PROMPT,CINPUT)
C
C---- 4-byte character string input
C     converted to uppercase
C
      CHARACTER*(*) PROMPT
      CHARACTER*4 CINPUT
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.LE.0) NP = LEN(PROMPT)
C
      WRITE(*,1000) PROMPT(1:NP)
      READ (*,1020) CINPUT
      CALL LC2UC(CINPUT)
C
      RETURN
C
 1000 FORMAT(/A,'   c>  ',$)
 1020 FORMAT(A4)
      END ! ASKC


      SUBROUTINE ASKC2(PROMPT,COMAND,CARGS)
C
C---- returns 4-byte character string input converted to uppercase
C---- also returns rest of input characters in CARGS string
C
      CHARACTER*(*) PROMPT
      CHARACTER*(*) COMAND, CARGS
C
      CHARACTER*128 LINE
C
      IZERO = ICHAR('0')
C
      NP = INDEX(PROMPT,'^') - 1
      IF(NP.EQ.0) NP = LEN(PROMPT)
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
      END ! ASKC2


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


      SUBROUTINE STRIP(STRING,NS)
      CHARACTER*(*) STRING
C-------------------------------------------
C     Strips leading blanks off string
C     and returns length of non-blank part.
C-------------------------------------------
      N = LEN(STRING)
C
C---- find last non-blank character
      DO 10 K2=N, 1, -1
        IF(STRING(K2:K2).NE.' ') GO TO 11
   10 CONTINUE
      K2 = 0
   11 CONTINUE
C
C---- find first non-blank character
      DO 20 K1=1, K2
        IF(STRING(K1:K1).NE.' ') GO TO 21
   20 CONTINUE
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
      DO 30 K=NS+1, N
        STRING(K:K) = ' '
   30 CONTINUE
C
      RETURN
      END
