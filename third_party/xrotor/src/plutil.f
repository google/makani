C***********************************************************************
C    Module:  plutil.f
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

      SUBROUTINE PLTINI(SCRNFR,IPSLU,IDEV,FACTOR,LPLOT,LLAND)
      LOGICAL LPLOT,LLAND
C
C---- terminate old plot if any
      IF(LPLOT) CALL PLEND
C
C---- initialize new plot
      IF(LLAND) THEN
        SIGNFR =  SCRNFR
      ELSE
        SIGNFR = -SCRNFR
      ENDIF
      CALL PLOPEN(SIGNFR,IPSLU,IDEV)
      LPLOT = .TRUE.
C
cC---- set X-window size in inches (might have been resized by user)
c      CALL GETWINSIZE(XWIND,YWIND)
cC
cC---- draw plot page outline offset by margins
c      CALL NEWPEN(5)
c      IF(XMARG .GT. 0.0) THEN
c        CALL PLOTABS(      XMARG,      YMARG,3)
c        CALL PLOTABS(      XMARG,YPAGE-YMARG,2)
c        CALL PLOTABS(XPAGE-XMARG,      YMARG,3)
c        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
c      ENDIF
c      IF(YMARG .GT. 0.0) THEN
c        CALL PLOTABS(      XMARG,      YMARG,3)
c        CALL PLOTABS(XPAGE-XMARG,      YMARG,2)
c        CALL PLOTABS(      XMARG,YPAGE-YMARG,3)
c        CALL PLOTABS(XPAGE-XMARG,YPAGE-YMARG,2)
c      ENDIF
c      CALL NEWPEN(1)
cC
c      CALL PLOTABS(XMARG,YMARG,-3)
c      CALL NEWCLIPABS( XMARG, XPAGE-XMARG, YMARG, YPAGE-YMARG )
C
      CALL NEWFACTOR(FACTOR)
C
      RETURN
      END


      SUBROUTINE COLORSPECTRUMHUES1(ncols,HUESTR)
      character*(*) HUESTR
C
C...Sets up a color "Spectrum" table that gives a continuous 
C   blend between a small number of base colors specified in the
C   character string HUESTR, which can be "RYGCBM" or any subset thereof.
C
C   The RGB components associated with each specified color are set in 
C   the DATA statement below. These colors are appended to any existing 
C   colormap data, typically set up by COLORMAPDEFAULT.
C
C   These Spectrum colors are then accessible by NEWCOLOR(-icol)
C       -icol = 1 .. ncols
C
C NOTE: The maximum number of colors available to the Spectrum is LESS 
C       than the screen depth would indicate.  Some of the X colormap 
C       is used by other X window applications, typically this will be 
C       around 30-40 colormap entries. So, for an 8 bit depth, this 
C       leaves around 220 or so available for use, only 210 or so after 
C       the Palette colors (typ. 10) are assigned.  Less are available 
C       if other applications are using the X colormap.
C
C...RGB components of the Spectrum-defining base colors
C   COLWIDTH controls the relative extent of that defining color
C
      parameter (NRGB = 7)
      dimension irgbhue(3,NRGB), huewidth(NRGB)
C
      DIMENSION IRGBTABLE(3,NRGB)
      DIMENSION COLORWIDTH(NRGB)
      CHARACTER*(NRGB) COLORCHARS
c
      DATA COLORCHARS / 'MBCGYOR' /
      DATA ( (IRGBTABLE(L,I),L=1,3),COLORWIDTH(I), I=1, NRGB )
     &  / 240,     0,   240,  1.5,    ! Magenta
     &     32,    32,   255,  1.0,    ! Blue
     &      0,   240,   240,  1.2,    ! Cyan
     &     32,   255,    32,  0.5,    ! Green
     &    240,   240,     0,  1.4,    ! Yellow
     &    255,   160,     0,  1.0,    ! Orange
     &    255,    32,    32,  1.2  /  ! Red
C         Red   Green    Blue
C
      call convrt2uc(HUESTR)
      nhuemax = len(HUESTR)
c
      nhue = 0
      do k=1, nhuemax
        i = index( COLORCHARS , HUESTR(k:k) )
        if(i.ne.0) then
         nhue = nhue + 1
         irgbhue(1,nhue) = IRGBTABLE(1,i)
         irgbhue(2,nhue) = IRGBTABLE(2,i)
         irgbhue(3,nhue) = IRGBTABLE(3,i)
         huewidth(nhue)  = COLORWIDTH(i)
        endif
      enddo
c
      CALL COLORSPECTRUMTRP(ncols,nhue,irgbhue,huewidth)
C
      return
      end



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
      X = XC + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YC + CHX*(CHDX*SINA + CHDY*COSA)
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
      X = XC + CHX*(CHDX*COSA - CHDY*SINA)
      Y = YC + CHX*(CHDX*SINA + CHDY*COSA)
      CALL PLFONT(X,Y,CHX*CHFAC,STRING,ANGLE,NC)
C
      RETURN
      END



      SUBROUTINE SCALIT(II,Y,YOFF,YSF)
      DIMENSION Y(II)
C.............................................................
C
C     Y(1:II)  array whose scaling factor is to be determined
C     YOFF     offset of Y array  (Y-YOFF is actually scaled)
C     YSF      Y scaling factor:
C
C            YSF * max(Y(i)-YOFF)  will be < 1, and O(1)
C     hence, 1/YSF is a good max axis-annotation value.
C.............................................................
C
      AG2 = ALOG10(2.0)
      AG5 = ALOG10(5.0)
C
      YMAX = ABS(Y(1) - YOFF)
      DO 10 I=2, II
        YMAX = MAX( YMAX , ABS(Y(I)-YOFF) )
   10 CONTINUE
C
      IF(YMAX .EQ. 0.0) THEN
        YSF = 1.0E8
        RETURN
      ENDIF
C
      YLOG = ALOG10(YMAX)
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
      GMIN = AMIN1( YLOG1 , YLOG2 , YLOG5 )
C
C---- set scaling factor
      YSF = 10.0**(-GMIN)
C
      RETURN
      END ! SCALIT


      SUBROUTINE PLCIRC(X0,Y0,RAD, NSEG)
      DTR = ATAN(1.0) / 45.0
C
      IF(NSEG.EQ.0) THEN
C------ draw solid circle
        CALL PLOT(X0+RAD,Y0,3)
        DO I=1, 360
          T = FLOAT(I) * DTR
          CALL PLOT(X0+RAD*COS(T),Y0+RAD*SIN(T),2)
        ENDDO
      ELSE
C------ draw dashed circle in NSEG segments
        KSEG = 360/NSEG
        IGAP = MAX( KSEG/5 , 1 )
        DO ISEG=1, NSEG
          I1 = (ISEG-1)*KSEG  + IGAP
          I2 =  ISEG   *KSEG  - IGAP
          IPEN = 3
          DO I=I1, I2
            T = FLOAT(I) * DTR
            CALL PLOT(X0+RAD*COS(T),Y0+RAD*SIN(T),IPEN)
            IPEN = 2
          ENDDO
        ENDDO
      ENDIF
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
       WRITE(*,*) 'x,y =', XX1/XSF+XOFF, YY1/YSF+YOFF
C
       CALL GETCURSORXY(XX2,YY2,KCHAR)
       CALL PLSYMB(XX2,YY2,SH,3,0.0,0)
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



      SUBROUTINE PABORT
      COMMON /COM_ABORT/ XABORT(2), YABORT(2)
C
      CALL GETWINSIZE(XWIND,YWIND)
      CALL GETORIGIN(XORG,YORG)
      CALL GETFACTORS(XSCALE,YSCALE)
C
C---- save and disable current clipping
      CALL GETCLIPABS(XMIN,XMAX,YMIN,YMAX)
      CALL CLRCLIP
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('red')
C
C---- set abort window in lower right corner
      XABORT(1) = (XWIND - 1.0 - XORG)/XSCALE
      XABORT(2) = (XWIND - 0.1 - XORG)/XSCALE
      YABORT(1) = (        0.1 - YORG)/YSCALE
      YABORT(2) = (        0.5 - YORG)/YSCALE
C
C---- plot abort window
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
C
C---- restore color and clipping
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIPABS(XMIN,XMAX,YMIN,YMAX)
C
      RETURN
      END



      SUBROUTINE PZOOM
      COMMON /COM_ZOOM/ XZOOM(2), YZOOM(2)
C
      CALL GETWINSIZE(XWIND,YWIND)
      CALL GETORIGIN(XORG,YORG)
      CALL GETFACTORS(XSCALE,YSCALE)
C
C---- save and disable current clipping
      CALL GETCLIPABS(XMIN,XMAX,YMIN,YMAX)
      CALL CLRCLIP
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('cyan')
C
C---- set zoom window in lower right corner
      XZOOM(1) = (XWIND - 2.0 - XORG)/XSCALE
      XZOOM(2) = (XWIND - 1.1 - XORG)/XSCALE
      YZOOM(1) = (        0.1 - YORG)/YSCALE
      YZOOM(2) = (        0.5 - YORG)/YSCALE
C
C---- plot zoom window
      CALL PLOT(XZOOM(1),YZOOM(1),3)
      CALL PLOT(XZOOM(2),YZOOM(1),2)
      CALL PLOT(XZOOM(2),YZOOM(2),2)
      CALL PLOT(XZOOM(1),YZOOM(2),2)
      CALL PLOT(XZOOM(1),YZOOM(1),2)
C
      CHA = MIN( (XZOOM(2)-XZOOM(1))/8.0 , (YZOOM(2)-YZOOM(1))/1.5 )
      XCA = 0.5*(XZOOM(2)+XZOOM(1)) - 2.0*CHA
      YCA = 0.5*(YZOOM(2)+YZOOM(1)) - 0.5*CHA
      CALL PLCHAR(XCA,YCA,CHA,'ZOOM',0.0,4)
C
C---- restore color and clipping
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIPABS(XMIN,XMAX,YMIN,YMAX)
C
      RETURN
      END

      SUBROUTINE PUNZOOM
      COMMON /COM_UNZOOM/ XUNZOOM(2), YUNZOOM(2)
C
      CALL GETWINSIZE(XWIND,YWIND)
      CALL GETORIGIN(XORG,YORG)
      CALL GETFACTORS(XSCALE,YSCALE)
C
C---- save and disable current clipping
      CALL GETCLIPABS(XMIN,XMAX,YMIN,YMAX)
      CALL CLRCLIP
C
      CALL GETCOLOR(ICOL0)
      CALL NEWCOLORNAME('cyan')
C
C---- set unzoom window in lower right corner
      XUNZOOM(1) = (XWIND - 3.0 - XORG)/XSCALE
      XUNZOOM(2) = (XWIND - 2.1 - XORG)/XSCALE
      YUNZOOM(1) = (        0.1 - YORG)/YSCALE
      YUNZOOM(2) = (        0.5 - YORG)/YSCALE
C
C---- plot unzoom window
      CALL PLOT(XUNZOOM(1),YUNZOOM(1),3)
      CALL PLOT(XUNZOOM(2),YUNZOOM(1),2)
      CALL PLOT(XUNZOOM(2),YUNZOOM(2),2)
      CALL PLOT(XUNZOOM(1),YUNZOOM(2),2)
      CALL PLOT(XUNZOOM(1),YUNZOOM(1),2)
C
      CHA = MIN( (XUNZOOM(2)-XUNZOOM(1))/9.0 , 
     &           (YUNZOOM(2)-YUNZOOM(1))/1.5 )
      XCA = 0.5*(XUNZOOM(2)+XUNZOOM(1)) - 3.0*CHA
      YCA = 0.5*(YUNZOOM(2)+YUNZOOM(1)) - 0.5*CHA
      CALL PLCHAR(XCA,YCA,CHA,'UNZOOM',0.0,6)
C
C---- restore color and clipping
      CALL NEWCOLOR(ICOL0)
      CALL NEWCLIPABS(XMIN,XMAX,YMIN,YMAX)
C
      RETURN
      END


      FUNCTION LABORT(XC,YC)
      LOGICAL LABORT
      COMMON /COM_ABORT/ XABORT(2), YABORT(2)
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


      FUNCTION LZOOM(XC,YC)
      LOGICAL LZOOM
      COMMON /COM_ZOOM/ XZOOM(2), YZOOM(2)
C
C---- return T if location XC,YC falls within zoom-select window
C
      LZOOM = XC .GE. XZOOM(1) .AND. 
     &        XC .LE. XZOOM(2) .AND.
     &        YC .GE. YZOOM(1) .AND.
     &        YC .LE. YZOOM(2)
C
      RETURN
      END


      FUNCTION LUNZOOM(XC,YC)
      LOGICAL LUNZOOM
      COMMON /COM_UNZOOM/ XUNZOOM(2), YUNZOOM(2)
C
C---- return T if location XC,YC falls within zoom-select window
C
      LUNZOOM = XC .GE. XUNZOOM(1) .AND. 
     &          XC .LE. XUNZOOM(2) .AND.
     &          YC .GE. YUNZOOM(1) .AND.
     &          YC .LE. YUNZOOM(2)
C
      RETURN
      END



      SUBROUTINE OPLSET(IDEV,IDEVRP,IPSLU,
     &                  SIZE,PAR,
     &                  XMARG,YMARG,XPAGE,YPAGE,
     &                  CSIZE,SCRNFR,LCURS,LLAND,LSLOPE)
      LOGICAL LCURS,LLAND,LSLOPE
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
     &              CHCURS, CHLAND, LSLOPE, LCOLOR
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
     &      /'  X slope matching on inputs   ', L2
     &      /'  C olor PostScript output?    ', L2 )
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
      ELSEIF (INDEX('Xx',VAR).NE.0) THEN
        LSLOPE = .NOT. LSLOPE
C
      ELSEIF (INDEX('Cc',VAR).NE.0) THEN
        LCOLOR = .NOT. LCOLOR
        IF(     LCOLOR) IDEVRP = 4
        IF(.NOT.LCOLOR) IDEVRP = 2
C
      ELSE
        WRITE(*,*) '*** Item not recognized ***'
      ENDIF
      GO TO 1
C
      END ! OPLSET
