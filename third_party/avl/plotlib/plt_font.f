C***********************************************************************
C    Module: plt_font.f 
C 
C    Copyright (C) 1996 Harold Youngren, Mark Drela 
C 
C    This library is free software; you can redistribute it and/or
C    modify it under the terms of the GNU Library General Public
C    License as published by the Free Software Foundation; either
C    version 2 of the License, or (at your option) any later version.
C
C    This library is distributed in the hope that it will be useful,
C    but WITHOUT ANY WARRANTY; without even the implied warranty of
C    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
C    Library General Public License for more details.
C
C    You should have received a copy of the GNU Library General Public
C    License along with this library; if not, write to the Free
C    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
C 
C    Report problems to:    guppy@maine.com 
C                        or drela@mit.edu  
C***********************************************************************
C
C***********************************************************************
C --- Font-related utility routines
C
C--- in user-coordinates...
C      PLNUMB - formats and plots a number (using PLCHAR)
C      PLCHAR - plots characters using CHAR font (WYSIWYG ASCII)
C      PLSLAN - plots characters using SLAN font (slanted ASCII)
C      PLMATH - plots characters using MATH font (LaTex-like symbols)
C      PLSYMB - plots plot-point symbol
C
C--- similarly, in absolute coordinates
C      PLNUMBABS - formats and plots a number
C      PLCHARABS - plots characters using CHAR font (normal  ASCII)
C      PLSLANABS - plots characters using SLAN font (slanted ASCII)
C      PLMATHABS - plots characters using MATH font (LaTex-like symbols)
C      PLSYMBABS - plots plot-point symbol
C
C     Version 4.46 11/28/01
C
C     Notes:  These routines require the include files that define the 
C             stroked fonts CHAR, MATH, SLAN, SYMB.
C***********************************************************************



      subroutine PLNUMB(xc,yc,chx,FPN,ANGLE,ndig)
C----------------------------------------------------------------
C     Plots a floating-point number as a string of characters
C
C      xc,yc  = user coordinate position for plotting first character 
C               (lower left point)
C               if xc=999. the last x plotting position is used 
C               if yc=999. the last y plotting position is used 
C      chx    = character width in x (user coordinate length)
C      FPN    = floating-point number to be plotted
C      ANGLE  = angle of character string (degrees from x-axis)
C      ndig   = specification of the number of digits and the type
C               of numeric string to be plotted (integer)
C             > 0 = number of digits to the right of the decimal
C                   point to be plotted (last digit is rounded)
C             = 0 = rounded integer portion of fpn is plotted
C                   with a decimal point
C            = -1 = rounded integer portion of fpn is plotted
C                   without the decimal point
C            < -1 = rounded integer portion of fpn is plotted
C                   after having the least significant digits
C                   truncated (IABS(NDID)-1 digits are truncated)
C
C        CALLS:  GETFACTORS, PLNUMABS
C----------------------------------------------------------------
C
C---Convert user coordinates and size to absolute coordinates
      XABS = xc
      YABS = yc
      if (xc.NE.999.)  XABS = xusr2ABS(xc)
      if (yc.NE.999.)  YABS = yusr2ABS(yc)
      call GETFACTORS(xscale,yscale)
      CHXABS = xscale*chx
C---Call absolute coordinate routine
      CALL PLNUMBABS(XABS,YABS,CHXABS,FPN,ANGLE,ndig)
C
      RETURN
      END

      subroutine PLCHAR(xc,yc,chx,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with standard character font
C
C      xc,yc  = user coordinate position for first character in string
C               if xc=999. the last x plotting position is used 
C               if yc=999. the last y plotting position is used 
C      chx     = character width (user coordinates)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETFACTORS, PLCHARABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
C---Convert user coordinates and size to absolute coordinates
      XABS = xc
      YABS = yc
      if (xc.NE.999.)  XABS = xusr2ABS(xc)
      if (yc.NE.999.)  YABS = yusr2ABS(yc)
      call GETFACTORS(xscale,yscale)
      CHXABS = xscale*chx
C---Call absolute coordinate routine
      CALL PLCHARABS(XABS,YABS,CHXABS,STRING,ANGLE,nc)
C
      RETURN
      END

      subroutine PLSLAN(xc,yc,chx,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with slanted character font
C
C      xc,yc  = user coordinate position for first character in string
C               if xc=999. the last x plotting position is used 
C               if yc=999. the last y plotting position is used 
C      chx     = character width (user coordinates)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETFACTORS, PLCHARABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
C---Convert user coordinates and size to absolute coordinates
      XABS = xc
      YABS = yc
      if (xc.NE.999.)  XABS = xusr2ABS(xc)
      if (yc.NE.999.)  YABS = yusr2ABS(yc)
      call GETFACTORS(xscale,yscale)
      CHXABS = xscale*chx
C---Call absolute coordinate routine
      CALL PLSLANABS(XABS,YABS,CHXABS,STRING,ANGLE,nc)
C
      RETURN
      END

      subroutine PLMATH(xc,yc,chx,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with math character font
C
C      xc,yc  = user coordinate position for first character in string
C               if xc=999. the last x plotting position is used 
C               if yc=999. the last y plotting position is used 
C      chx     = character width (user coordinates)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETFACTORS, PLMATHABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
C---Convert user coordinates and size to absolute coordinates
      XABS = xc
      YABS = yc
      if (xc.NE.999.)  XABS = xusr2ABS(xc)
      if (yc.NE.999.)  YABS = yusr2ABS(yc)
      call GETFACTORS(xscale,yscale)
      CHXABS = xscale*chx
C---Call absolute coordinate routine
      CALL PLMATHABS(XABS,YABS,CHXABS,STRING,ANGLE,nc)
C
      RETURN
      END

      subroutine PLSYMB(xc,yc,chx,ISYM,ANGLE,nc)
C----------------------------------------------------------------
C     Plots a symbol with symbol font indexed by integer
C
C      xc,yc  = user coordinate position for plotting symbol
C               if xc=999. the last x plotting position is used 
C               if yc=999. the last y plotting position is used 
C      chx     = symbol width
C      ISYM   = integer to select symbol (0..?)
C      ANGLE  = angle for symbol (radians, positive is righthanded rotation)
C      nc     = 0  just move to x,y before plotting symbol
C           .ne.0  draw line to x,y before plotting symbol
C
C     Plots a symbol using vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 48,48)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETFACTORS, PLSYMBABS
C----------------------------------------------------------------
      INTEGER ISYM
C
C---Convert user coordinates and size to absolute coordinates
      XABS = xc
      YABS = yc
      if (xc.NE.999.)  XABS = xusr2ABS(xc)
      if (yc.NE.999.)  YABS = yusr2ABS(yc)
      call GETFACTORS(xscale,yscale)
      CHXABS = xscale*chx
C---Call absolute coordinate routine
      CALL PLSYMBABS(XABS,YABS,CHXABS,ISYM,ANGLE,nc)
C
      RETURN
      END






      subroutine PLNUMBABS(XC,YC,CHX,FPN,ANGLE,ndig)
C----------------------------------------------------------------
C     Plots a floating-point number as a string of characters
C
C      XC,YC  = absolute coordinate position for plotting first character 
C               (lower left point)
C               if XC=999. the last x plotting position is used 
C               if YC=999. the last y plotting position is used 
C      CHX    = character width in x (absolute coordinate length)
C      FPN    = floating-point number to be plotted
C      ANGLE  = angle of character string (degrees from x-axis)
C      ndig   = specification of the number of digits and the type
C               of numeric string to be plotted (integer)
C             > 0 = number of digits to the right of the decimal
C                   point to be plotted (last digit is rounded)
C             = 0 = rounded integer portion of fpn is plotted
C                   with a decimal point
C            = -1 = rounded integer portion of fpn is plotted
C                   without the decimal point
C            < -1 = rounded integer portion of fpn is plotted
C                   after having the least significant digits
C                   truncated (IABS(NDID)-1 digits are truncated)
C
C        CALLS:  PLCHARABS
C----------------------------------------------------------------
      CHARACTER*1 MINUS, POINT, CHDIG
      DATA MINUS/'-'/, POINT/'.'/
C
      IZERO = ichar('0')
C
      T1 = FPN
      XX = XC
      YY = YC
C...Number negative? 
      IF (T1 .LT. 0.0) THEN
        CALL PLCHARABS(XX,YY,CHX,MINUS,ANGLE,1)
        xx = 999.
        yy = 999.
        T1 = -T1
      ENDIF
C
C...Set working digit count
      ND = -ndig
C
C...Integer only to be plotted? 
      IF (ndig.LE.0) THEN
C...Round and truncate for integer
        IF(ndig.EQ.0)  ND = 1
        ND = ND - 1
        T2 = FLOAT(IFIX((T1 + 0.5)/(10.**ND))) + 0.5
        ND = 0
        IF (ndig.EQ.0) ND = -1
       ELSE
C...Round for fraction
        T2 = T1 + 0.5/(10.**ndig)
      ENDIF
C
C...Find number of digits to the left of decimal point
      NL = 1
C...Any more digits to the left of the d.p.
   60 IF (T2 .GE. 10.) THEN
        T2 = T2/10.
        NL = NL + 1
        GO TO 60
      ENDIF
C
C...Set plottable digit count
      NP = NL - ND
C...Bad digit count?
      IF (NP.LE.0) NP = 1
C
C...Plot decimal point (NL=0)
   80 IF(NL.EQ.0) THEN
C...No decimal point?
        IF (ndig.LT.0) GO TO 120
        CALL PLCHARABS(XX,YY,CHX,POINT,ANGLE,1)
        IF (ndig.NE.0) NP = NP + 1
       ELSE
C...Plot digit
        IDIG = IFIX(T2)
        T2 = (T2 - FLOAT (IDIG))*10.
        CHDIG = char(IDIG + IZERO)
        CALL PLCHARABS(XX,YY,CHX,CHDIG,ANGLE,1)
      ENDIF
      xx = 999.
      yy = 999.
C
C...Count digit
      NP = NP - 1
  120 NL = NL - 1
C
C...More digits to plot?
      IF(NP.GT.0) GO TO 80
      RETURN
      END

      subroutine PLCHARABS(XC,YC,CHX,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with standard character font
C
C      XC,YC  = absolute coordinate position for first character in string
C               if XC=999. the last x plotting position is used 
C               if YC=999. the last y plotting position is used 
C      CHX    = character width in x (absolute coordinate length)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETLASTXYABS, GETPAT, NEWPAT, PLOTABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
      INCLUDE 'CHAR.INC'
C
      DATA  PI /3.1415926535897932384/
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C---- if XC=999. or YC=999. get last x,y character plotting position
      IF (xc.EQ.999. .OR. yc.EQ.999.) THEN
        CALL GETLASTXYABS(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
C---- get the old line pattern, only plot characters & symbols with solid lines
      CALL GETPAT(lmask)
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(-1)
      ENDIF
C
C---- if number of characters
      ncc = nc
      if(ncc.LT.0) ncc = LEN(STRING)
      if(ncc.EQ.0) return
c
C---- go over each character...
      DO 12 IC=1, ncc
C
C------ set character origin location
        X0 = XX + CHX*FLOAT(IC-1)*COSA
        Y0 = YY + CHX*FLOAT(IC-1)*SINA
C
        KC = INDEX(CHARS,STRING(IC:IC))
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
            FI = FLOAT(I-16)/64.0
            FJ = FLOAT(J-16)/64.0
C
            X = X0 + CHX*(FI*COSA - FJ*SINA)
            Y = Y0 + CHX*(FI*SINA + FJ*COSA)
C
            IF     (ISTAT.EQ.0) THEN
             GOTO 12
            ELSEIF (ISTAT.EQ.1) THEN
             CALL PLOTABS(X,Y,2)
            ELSE
             CALL PLOTABS(X,Y,3)
            ENDIF
C
          ENDDO
        ENDIF
C
 12   CONTINUE
C
C---- move to bottom right corner x,y for next call with x,y = 999.,999.
      X0 = XX + CHX*FLOAT(ncc)*COSA
      Y0 = YY + CHX*FLOAT(ncc)*SINA
      CALL PLOTABS(X0,Y0,3)
C
C---- restore line pattern
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(lmask)
      ENDIF
C
      RETURN
      END

      subroutine PLSLANABS(XC,YC,CHX,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with slanted character font
C
C      XC,YC  = absolute coordinate position for first character in string
C               if XC=999. the last x plotting position is used 
C               if YC=999. the last y plotting position is used 
C      CHX    = character width in x (absolute coordinate length)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETLASTXYABS, GETPAT, NEWPAT, PLOTABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
      INCLUDE 'SLAN.INC'
C
      DATA  PI /3.1415926535897932384/
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C---- if XC=999. or YC=999. get last x,y character plotting position
      IF (xc.EQ.999. .OR. yc.EQ.999.) THEN
        CALL GETLASTXYABS(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
C---- get the old line pattern, only plot characters & symbols with solid lines
      CALL GETPAT(lmask)
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(-1)
      ENDIF
C
C---- if number of characters
      ncc = nc
      if(ncc.LT.0) ncc = LEN(STRING)
      if(ncc.EQ.0) return
c
C---- go over each character...
      DO 12 IC=1, ncc
C
C------ set character origin location
        X0 = XX + CHX*FLOAT(IC-1)*COSA
        Y0 = YY + CHX*FLOAT(IC-1)*SINA
C
        KC = INDEX(CHARS,STRING(IC:IC))
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
            FI = FLOAT(I-16)/64.0
            FJ = FLOAT(J-16)/64.0
C
            X = X0 + CHX*(FI*COSA - FJ*SINA)
            Y = Y0 + CHX*(FI*SINA + FJ*COSA)
C
            IF     (ISTAT.EQ.0) THEN
             GOTO 12
            ELSEIF (ISTAT.EQ.1) THEN
             CALL PLOTABS(X,Y,2)
            ELSE
             CALL PLOTABS(X,Y,3)
            ENDIF
C
          ENDDO
        ENDIF
C
 12   CONTINUE
C
C---- move to bottom right corner x,y for next call with x,y = 999.,999.
      X0 = XX + CHX*FLOAT(ncc)*COSA
      Y0 = YY + CHX*FLOAT(ncc)*SINA
      CALL PLOTABS(X0,Y0,3)
C
C---- restore line pattern
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(lmask)
      ENDIF
C
      RETURN
      END

      subroutine PLMATHABS(XC,YC,CHX,STRING,ANGLE,nc)
C----------------------------------------------------------------
C     Plots character string with math character font
C
C      XC,YC  = absolute coordinate position for first character in string
C               if XC=999. the last x plotting position is used 
C               if YC=999. the last y plotting position is used 
C      CHX    = character width in x (absolute coordinate length)
C      STRING = character string to plot with nc characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      nc     = number of characters to plot
C               if nc<0 the length of the string is determined automatically 
C
C     Character plotting uses the vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 16,16)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETLASTXYABS, GETPAT, NEWPAT, PLOTABS
C----------------------------------------------------------------
      CHARACTER*(*) STRING
C
      INCLUDE 'MATH.INC'
C
      DATA  PI /3.1415926535897932384/
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C---- if XC=999. or YC=999. get last x,y character plotting position
      IF (xc.EQ.999. .OR. yc.EQ.999.) THEN
        CALL GETLASTXYABS(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
C---- get the old line pattern, only plot characters & symbols with solid lines
      CALL GETPAT(lmask)
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(-1)
      ENDIF
C
C---- if number of characters
      ncc = nc
      if(ncc.LT.0) ncc = LEN(STRING)
      if(ncc.EQ.0) return
c
C---- go over each character...
      DO 12 IC=1, ncc
C
C------ set character origin location
        X0 = XX + CHX*FLOAT(IC-1)*COSA
        Y0 = YY + CHX*FLOAT(IC-1)*SINA
C
        KC = INDEX(CHARS,STRING(IC:IC))
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
            FI = FLOAT(I-16)/64.0
            FJ = FLOAT(J-16)/64.0
C
            X = X0 + CHX*(FI*COSA - FJ*SINA)
            Y = Y0 + CHX*(FI*SINA + FJ*COSA)
C
            IF     (ISTAT.EQ.0) THEN
             GOTO 12
            ELSEIF (ISTAT.EQ.1) THEN
             CALL PLOTABS(X,Y,2)
            ELSE
             CALL PLOTABS(X,Y,3)
            ENDIF
C
          ENDDO
        ENDIF
C
 12   CONTINUE
C
C---- move to bottom right corner x,y for next call with x,y = 999.,999.
      X0 = XX + CHX*FLOAT(ncc)*COSA
      Y0 = YY + CHX*FLOAT(ncc)*SINA
      CALL PLOTABS(X0,Y0,3)
C
C---- restore line pattern
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(lmask)
      ENDIF
C
      RETURN
      END

      subroutine PLSYMBABS(XC,YC,CHX,ISYM,ANGLE,nc)
C----------------------------------------------------------------
C     Plots a symbol with symbol font indexed by integer
C
C      XC,YC  = absolute coordinate position for plotting symbol
C               if XC=999. the last x plotting position is used 
C               if YC=999. the last y plotting position is used 
C      CHX    = symbol width in x (absolute coordinate length)
C      ISYM   = integer to select symbol (0..?)
C      ANGLE  = angle for symbol (radians, positive is righthanded rotation)
C      nc     = 0  just move to x,y before plotting symbol
C           .ne.0  draw line to x,y before plotting symbol
C
C     Plots a symbol using vector font database NODE(..)
C     Each NODE(..) value has the form  sxxyy
C
C        xx   = location of polyline point   0 ... 96
C        yy       (origin assumed to be at 48,48)
C        s    = 2  if point is the start of a new polyline stroke
C             = 1  if point is inside or at the end of stroke
C             = 0  if point is not valid
C
C        CALLS:  GETLASTXYABS, GETPAT, NEWPAT, PLOTABS
C----------------------------------------------------------------
      INTEGER ISYM
C
      INCLUDE 'SYMB.INC'
C
      DATA  PI /3.1415926535897932384/
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C---- if XC=999. or YC=999. get last x,y character plotting position
      IF (XC.EQ.999. .OR. YC.EQ.999.) THEN
        CALL GETLASTXYABS(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
C------ set character origin location
      X0 = XX
      Y0 = YY
C------ draw to X0,Y0 first ?
      IF(nc .NE. 0) CALL PLOTABS(X0,Y0,2)
C
C---- get the old line pattern, only plot characters & symbols with solid lines
      CALL GETPAT(lmask)
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(-1)
      ENDIF
C
C
C---Plot symbol index modulo symbol count (NCHARS defined symbols)
ccc      KC = ISYM + 1
      KC = MOD(ISYM,NCHARS) + 1
C
      IF(KC.GE.1 .AND. KC.LE.NCHARS) THEN
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
          FI = FLOAT(I-48)/64.0
          FJ = FLOAT(J-48)/64.0
C
          X = X0 + CHX*(FI*COSA - FJ*SINA)
          Y = Y0 + CHX*(FI*SINA + FJ*COSA)
C
          IF (ISTAT.EQ.0) THEN
            GOTO 12
           ELSEIF (ISTAT.EQ.1) THEN
            CALL PLOTABS(X,Y,2)
           ELSE
            CALL PLOTABS(X,Y,3)
          ENDIF
C
        ENDDO
      ENDIF
C
 12   CONTINUE
C
C---- move to origin x,y for next call with x,y = 999.,999.
      CALL PLOTABS(X0,Y0,3)
C
C---- restore line pattern
      IF(lmask.ne.-1) THEN
        CALL NEWPAT(lmask)
      ENDIF
C
      RETURN
      END

