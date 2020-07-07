C***********************************************************************
C    Module:  plutil.f
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

      SUBROUTINE OPLSET(IDEV,IDEVRP,IPSLU,
     &                  SIZE,PAR,
     &                  XMARG,YMARG,XPAGE,YPAGE,
     &                  CSIZE,SCRNFR,LCURS,LLAND, ICOLS)
      LOGICAL LCURS,LLAND
C-----------------------------------------------------------
C     Allows user modification of various plot parameters.
C-----------------------------------------------------------
      CHARACTER*1 VAR
      CHARACTER*4 COMAND
      CHARACTER*128 COMARG
      CHARACTER*10 CHCURS, CHLAND
      DIMENSION IINPUT(20)
      DIMENSION RINPUT(20)
      LOGICAL ERROR, LGRAPH, LCOLOR
      INTEGER ICOLS(2)
C
 1000 FORMAT(A)
C
 1    CONTINUE
      IF(LCURS) THEN
       CHCURS = 'Cursor    '
      ELSE
       CHCURS = 'Keyboard  '
      ENDIF
C
      IF(LLAND) THEN
       CHLAND = 'Landscape '
      ELSE
       CHLAND = 'Portrait  '
      ENDIF
C
      LGRAPH = IDEV  .GE.1
      LCOLOR = IDEVRP.EQ.4
C
      WRITE(*,2000) LGRAPH, SIZE, PAR,
     &              XPAGE,YPAGE, XMARG,YMARG, 
     &              CSIZE, SCRNFR,
     &              CHCURS, CHLAND, LCOLOR,
     &              ICOLS(1), ICOLS(2)
 2000 FORMAT(' ...............................................'
     &     //'  G raphics-enable flag:       ', L2,
     &      /'  S ize of plot object         ', F6.2,'"'
     &      /'  A spect ratio of plot object ', F8.4
     &      /'  P age dimensions             ', F6.2,' x',F6.2,'"'
     &      /'  M argins from page edges     ', F6.2,'",',F6.2,'"'
     &      /'  F ont size (relative)        ', F8.4
     &      /'  W indow/screen size fraction ', F8.4
     &      /'  B lowup input method:        ', A 
     &      /'  O rientation of plot:        ', A 
     &      /'  C olor PostScript output?    ', L2,
     &      /'  L ine colors for top,bottom  ', 2I3 )
C
 5    CALL ASKC('      Option, Value   (or <Return>) ^',COMAND,COMARG)
C
      DO I=1, 20
        IINPUT(I) = 0.0
        RINPUT(I) = 0.0
      ENDDO
      NINPUT = 0
      CALL GETINT(COMARG,IINPUT,NINPUT,ERROR)
      NINPUT = 0
      CALL GETFLT(COMARG,RINPUT,NINPUT,ERROR)
C
      VAR = COMAND(1:1)
      IF (VAR.EQ.'0' .OR. VAR.EQ.' ') THEN
       RETURN
C
      ELSEIF (INDEX('Gg',VAR).NE.0) THEN
        IF(IDEV.EQ.0) THEN
         IDEV = 1
        ELSE
         IDEV = 0
        ENDIF
C
      ELSEIF (INDEX('Ss',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          SIZE = RINPUT(1)
        ELSE
          CALL ASKR('Enter size (in)^',SIZE)
        ENDIF
C
      ELSEIF (INDEX('Aa',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          PAR = RINPUT(1)
        ELSE
          CALL ASKR('Enter aspect ratio^',PAR)
        ENDIF
C
      ELSEIF (INDEX('Pp',VAR).NE.0) THEN
        IF(NINPUT.GE.2) THEN
          XPAGE = RINPUT(1)
          YPAGE = RINPUT(2)
        ELSEIF(NINPUT.GE.1) THEN
          XPAGE = RINPUT(1)
          CALL ASKR('Enter page Y dimension (in)^',YPAGE)
        ELSE
          CALL ASKR('Enter page X dimension (in)^',XPAGE)
          CALL ASKR('Enter page Y dimension (in)^',YPAGE)
        ENDIF
C
      ELSEIF (INDEX('Mm',VAR).NE.0) THEN
        IF(NINPUT.GE.2) THEN
          XMARG = RINPUT(1)
          YMARG = RINPUT(2)
        ELSEIF(NINPUT.GE.1) THEN
          XMARG = RINPUT(1)
          CALL ASKR('Enter page Y margin (in)^',YMARG)
        ELSE
          CALL ASKR('Enter page X margin (in)^',XMARG)
          CALL ASKR('Enter page Y margin (in)^',YMARG)
        ENDIF
C
      ELSEIF (INDEX('Ff',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          CSIZE = RINPUT(1)
        ELSE
          CALL ASKR('Enter character font size^',CSIZE)
        ENDIF
C
      ELSEIF (INDEX('Ww',VAR).NE.0) THEN
        IF(NINPUT.GE.1) THEN
          SCRNFR = RINPUT(1)
        ELSE
          CALL ASKR('Enter window/screen size fraction^',SCRNFR)
        ENDIF
C
      ELSEIF (INDEX('Bb',VAR).NE.0) THEN
        LCURS = .NOT. LCURS
C
      ELSEIF (INDEX('Oo',VAR).NE.0) THEN
        LLAND = .NOT. LLAND
        WRITE(*,*)
        WRITE(*,*) 'Swapping X,Y page dimensions'
        XTMP = XPAGE
        YTMP = YPAGE
        XPAGE = YTMP
        YPAGE = XTMP
C
      ELSEIF (INDEX('Cc',VAR).NE.0) THEN
        LCOLOR = .NOT. LCOLOR
        IF(     LCOLOR) IDEVRP = 4
        IF(.NOT.LCOLOR) IDEVRP = 2
C
      ELSEIF (INDEX('Ll',VAR).NE.0) THEN
C------ set top and bottom-side colors
        IF(NINPUT.GE.2) THEN
          ICOLS(1) = IINPUT(1)
          ICOLS(2) = IINPUT(2)
        ELSE
 20      WRITE(*,2200)
 2200    FORMAT(
     &  /'  1  black  '
     &  /'  2  white  '
     &  /'  3  red    '
     &  /'  4  orange ' 
     &  /'  5  yellow ' 
     &  /'  6  green  '
     &  /'  7  cyan   '
     &  /'  8  blue   '
     &  /'  9  violet '
     &  /' 10  magenta' )
C
        WRITE(*,*) 'Select top,bottom line-plot colors'
        READ(*,*,ERR=20) ICOLS(1), ICOLS(2)
       ENDIF

       ICOLS(1) = MAX( 1, MIN( 10, ICOLS(1) ) )
       ICOLS(2) = MAX( 1, MIN( 10, ICOLS(2) ) )

      ELSE
        WRITE(*,*) '*** Item not recognized ***'
      ENDIF
      GO TO 1
C
      END ! OPLSET


      SUBROUTINE PLSUBS(XC,YC,CHX,STRING,ANGLE,NC,PLFONT)
C----------------------------------------------------------------
C     Plots character string as a subscript with font routine PLFONT.
C
C      XC,YC  = user coordinates of character to be subscripted 
C      CHX    = character width (user coordinates)
C      STRING = subscript character string to plot with NC characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      NC     = number of subscript characters to plot
C               if NC<0 the length of the string is determined automatically 
C----------------------------------------------------------------
      CHARACTER*(*) STRING
      EXTERNAL PLFONT
      DATA  PI /3.1415926535897932384/
C
C---- subscript character reduction factor, and x,y-shift/chx
      DATA CHFAC, CHDX, CHDY / 0.7, 0.9, -0.4 /
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C
      IF (XC.EQ.999. .OR. YC.EQ.999.) THEN
        CALL GETLASTXY(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
      X = XX + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YY + CHX*(CHDX*SINA + CHDY*COSA)
      CALL PLFONT(X,Y,CHX*CHFAC,STRING,ANGLE,NC)
C
      RETURN
      END



      SUBROUTINE PLSUPS(XC,YC,CHX,STRING,ANGLE,NC,PLFONT)
C----------------------------------------------------------------
C     Plots character string as a superscript with font routine PLFONT.
C
C      XC,YC  = user coordinates of character to be superscripted
C      CHX    = character width (user coordinates)
C      STRING = superscript character string to plot with NC characters
C      ANGLE  = angle of character (radians, positive is righthanded rotation)
C      NC     = number of superscript characters to plot
C               if NC<0 the length of the string is determined automatically 
C----------------------------------------------------------------
      CHARACTER*(*) STRING
      EXTERNAL PLFONT
      DATA  PI /3.1415926535897932384/
C
C---- superscript character reduction factor, and x,y-shift/chx
      DATA CHFAC, CHDX, CHDY / 0.7, 0.95, 0.7 /
C
      SINA = SIN(ANGLE*PI/180.0)
      COSA = COS(ANGLE*PI/180.0)
C
      XX = XC
      YY = YC
C
      IF (XC.EQ.999. .OR. YC.EQ.999.) THEN
        CALL GETLASTXY(XCHR,YCHR)
        IF(XC.EQ.999.) XX = XCHR
        IF(YC.EQ.999.) YY = YCHR
      ENDIF
C
      X = XX + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YY + CHX*(CHDX*SINA + CHDY*COSA)
      CALL PLFONT(X,Y,CHX*CHFAC,STRING,ANGLE,NC)
C
      RETURN
      END



      SUBROUTINE SCALIT(II,Y,YOFF,YSF)
      DIMENSION Y(II)
C-------------------------------------------------------------
C     Y(1:II)  array whose scaling factor is to be determined
C     YOFF     offset of Y array  (Y-YOFF is actually scaled)
C     YSF      Y scaling factor
C-------------------------------------------------------------
C
      AG2 = LOG10(2.0)
      AG5 = LOG10(5.0)
C
      YMAX = ABS(Y(1) - YOFF)
      DO 10 I=2, II
        YMAX = MAX( YMAX , ABS(Y(I)-YOFF) )
   10 CONTINUE
C
      IF(YMAX .EQ. 0.0) YMAX = 1.0E-8
      YLOG = LOG10(YMAX)
C
C---- find log of nearest power of 10 above YMAX
      YLOG1 = AINT(YLOG+100.0) - 99.0
 
C---- find log of nearest 2x(power of 10) above YMAX
      YLOG2 = YLOG1 + AG2
      IF(YLOG2-1.0.GT.YLOG) YLOG2 = YLOG2 - 1.0
C
C---- find log of nearest 5x(power of 10) above YMAX
      YLOG5 = YLOG1 + AG5
      IF(YLOG5-1.0.GT.YLOG) YLOG5 = YLOG5 - 1.0
C
C---- find log of smallest upper bound
      GMIN = MIN( YLOG1 , YLOG2 , YLOG5 )
C
C---- set scaling factor
      YSF = 10.0**(-GMIN)
C
      RETURN
      END




      SUBROUTINE OFFGET(XOFF,YOFF,XSF,YSF,XWIND,YWIND,LSAME,LCURS)
      LOGICAL LSAME, LCURS
      CHARACTER*1 KCHAR
C---------------------------------------------------
C     Sets new blowup parameters from cursor input.
C---------------------------------------------------
C
C---- crosshair "+" symbol size
      DATA SH / 2.0 /
C
C---- get current color
      CALL GETCOLOR(ICOL0)
C
C---- set new crosshair color
      CALL NEWCOLORNAME('red')
C
C
      IF(LCURS) THEN
C
       WRITE(*,*)
       WRITE(*,*) 'Mark off corners of blowup area'
       WRITE(*,*) '(2 identical points default to current area)'
C
       CALL GETCURSORXY(XX1,YY1,KCHAR)
       CALL PLSYMB(XX1,YY1,SH,3,0.0,0)
       CALL PLFLUSH
       WRITE(*,*) 'x,y =', XX1/XSF+XOFF, YY1/YSF+YOFF
C
       CALL GETCURSORXY(XX2,YY2,KCHAR)
       CALL PLSYMB(XX2,YY2,SH,3,0.0,0)
       CALL PLFLUSH
       WRITE(*,*) 'x,y =', XX2/XSF+XOFF, YY2/YSF+YOFF
C
      ELSE
C
       WRITE(*,*)
       WRITE(*,*) 'Enter x,y coordinates of blowup area corners'
       WRITE(*,*) '(2 identical points default to current area)'
       WRITE(*,*)
    1  WRITE(*,*) 'Point 1: '
       READ(*,*,ERR=1) XX1, YY1
    2  WRITE(*,*) 'Point 2: '
       READ(*,*,ERR=2) XX2, YY2
C
      ENDIF
C
C---- restore to initial color
      CALL NEWCOLOR(icol0)
C
      IF(XX1.EQ.XX2 .AND. YY1.EQ.YY2) RETURN
C
C
      XCEN = 0.5*(XX1+XX2)/XSF + XOFF
      YCEN = 0.5*(YY1+YY2)/YSF + YOFF
      XDIF = ABS(XX2 - XX1)/XSF
      YDIF = ABS(YY2 - YY1)/YSF
C
      IF(XDIF.EQ.0.0) XDIF = 1.0E-5
      IF(YDIF.EQ.0.0) YDIF = 1.0E-5
C
      XOFF = MIN(XX1,XX2)/XSF + XOFF
      YOFF = MIN(YY1,YY2)/YSF + YOFF
      XSF = XWIND/XDIF
      YSF = YWIND/YDIF
C
      IF(LSAME) THEN
C------ set equal x,y scales
        SF = MIN( XSF , YSF )
        XSF = SF
        YSF = SF
C
C------ re-center the blowup
        XOFF = XCEN - 0.5*XDIF
        YOFF = YCEN - 0.5*YDIF
      ENDIF
C
      RETURN
      END ! OFFGET



      SUBROUTINE PGUI(KBOX,COLOR,LABEL)
      CHARACTER*(*) COLOR, LABEL
C
      CALL GETWINSIZE(XWIND,YWIND)
cc      CALL GETORIGIN(XORG,YORG)
cc      CALL GETFACTORS(XSCALE,YSCALE)
C
C---- save and disable current clipping
      CALL GETCLIPABS(XMIN,XMAX,YMIN,YMAX)
      CALL CLRCLIP
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME(COLOR)
C
C---- set click box in lower right corner
      YBOX = 0.5*FLOAT(KBOX-1)
      X1 = XWIND - 1.0
      X2 = XWIND - 0.1
      Y1 = YBOX  + 0.1
      Y2 = YBOX  + 0.5
cc      X1 = (XWIND - 1.0 - XORG)/XSCALE
cc      X2 = (XWIND - 0.1 - XORG)/XSCALE
cc      Y1 = (YBOX  + 0.1 - YORG)/YSCALE
cc      Y2 = (YBOX  + 0.5 - YORG)/YSCALE
C
      CALL GUIBOX(KBOX, X1,X2,Y1,Y2, COLOR, LABEL)
C
C---- restore color and clipping
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIPABS(XMIN,XMAX,YMIN,YMAX)
C
      RETURN
      END

 

      SUBROUTINE ARROW(X,Y,DX,DY)
      CALL PLOT(X,Y,3)
      CALL PLOT(X+DX,Y+DY,2)
      X1 = X + 0.85*DX + 0.02*DY
      Y1 = Y + 0.85*DY - 0.02*DX
      X2 = X + 0.85*DX - 0.02*DY
      Y2 = Y + 0.85*DY + 0.02*DX
      CALL PLOT(X1,Y1,2)
      CALL PLOT(X2,Y2,2)
      CALL PLOT(X+DX,Y+DY,2)
      RETURN
      END
 
 
      SUBROUTINE DASH(X1,X2,Y)
      CALL NEWPEN(1)
      DX = (X2-X1)/50.0
      DO 10 I=1, 51
        X = X1 + DX*FLOAT(I-1)
        CALL PLOT(X-0.08*DX,Y,3)
        CALL PLOT(X+0.08*DX,Y,2)
   10 CONTINUE
      RETURN
      END


