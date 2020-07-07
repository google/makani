C***********************************************************************
C    Module: plt_color.f 
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
C --- Xplot11 color plotting routines
C
C     Version 4.46 11/28/01
C
C     Note:  These routines implement the interface to setup, select and 
C            query colors in the XPLOT11 plot package.
C***********************************************************************
C
C   The default colormap defines these colors and associated color indices
C   (before the user defines any more)...
C        BLACK   =  1
C        WHITE   =  2
C        YELLOW  =  3
C        ORANGE  =  4
C        RED     =  5
C        GREEN   =  6
C        CYAN    =  7
C        BLUE    =  8
C        MAGENTA =  9
C        VIOLET  =  10


      subroutine NEWCOLOR(icol)
C...Sets color by composite color index
C   color is set by the map index (for +icol) 
C   or spectrum index (for -icol)
C     Color map indices      run from 0  ->  N_COLORS
C     Color spectrum indices run from -1 -> -N_SPECTRUM
C
C    (see colormap subroutines below for setting colormap colors)
C
      include 'pltlib.inc'
c
      if(icol.GT.0) then
        if(icol.GT.N_COLOR) then
          write(*,*) 'NEWCOLOR: color index out of bounds: ',
     &                icol,N_COLOR
          return
        endif
        icindex = icol
      else
        if(-icol.GT.N_SPECTRUM) then
          write(*,*) 'NEWCOLOR: spectrum index out of bounds: ',
     &                -icol,N_SPECTRUM
          return
        endif
        icindex = IFIRST_SPECTRUM - icol - 1
      endif
c
C...Skip if this is the current color 
      if(icindex.EQ.I_CLR) return
c
C...Install color command into display primitives list
      I_CLR = icindex
      call putprim(ColorCommand,I_CLR,0.,0.)
c
      return
      end


      subroutine GETCOLOR(icol)
C...Returns current foreground color composite index 
C   if icol>0 the index is the color table index (non-spectrum colors)
C   if icol<0 the index is -(color spectrum index) 
C
      include 'pltlib.inc'
      if(I_CLR.ge.IFIRST_SPECTRUM .and. 
     &   I_CLR.le.IFIRST_SPECTRUM+N_SPECTRUM-1) then
        icol = IFIRST_SPECTRUM - I_CLR - 1
      else
        icol = I_CLR
      endif
      return
      end


      subroutine GETCOLORINDEX(icindex)
C...Returns color table index (not spectrum color index) 
C   of current foreground color table index (icindex runs from 0 -> N_COLOR)
C
      include 'pltlib.inc'
      icindex = I_CLR
      return
      end


      subroutine NEWCOLORNAME(colorname)
C...Sets color for plotting by named string 
C   (to circumvent knowing the color table index)
C   Valid color names (either upper or lower case) are found by 
C   running the X11 command: showrgb
C
      character colorname*(*), colorin*22
      include 'pltlib.inc'
c
C...Convert input color to uppercase
      colorin = colorname
      call convrt2uc(colorin)
c
C...Search for color name in current colortable
      do ic = 1, N_COLOR
c       write(*,*) 'colorbyname table ic=',ic,' ',colorin,' ',
c     &             COLOR_NAME(ic),' ci ',G_COLOR_CINDEX(ic)
        if(colorin.eq.COLOR_NAME(ic)) then
          call NEWCOLOR(ic)
          return  
        endif
      end do
c
C...Add new color to colortable
C...Get RGB components for named color
      call gw_cname2rgb(colorname,ired,igreen,iblue)
c      write(*,*) 'cname->rgb ',colorname, ired,igreen,iblue
c
      if (ired.ge.0) then
        N = N_COLOR + 1
        if(N.gt.Ncolors_max) then
          write(*,*)
     &        'NEWCOLORNAME: Colortable overflow. New color ignored.'
          return
        endif
        G_COLOR_CINDEX(N) = -1
        COLOR_RGB(N)  = iblue + 256*(igreen + 256*ired)
        COLOR_NAME(N) = colorin
        N_COLOR = N
        call NEWCOLOR(N)
       else
        write(*,*)
     &        'NEWCOLORNAME: Color not found ',colorname
      endif
c
      return
      end


      subroutine NEWCOLORRGB(ired,igreen,iblue)
C...Sets color for plotting by R,G,B components
C   (to circumvent knowing the color table index)
C   Valid color components for red,green,blue run from 0-255
C
      include 'pltlib.inc'
c
C...Search for r,g,b color in current colortable
      do ic = 1, N_COLOR
        irgb = iblue + 256*(igreen + 256*ired)
c        write(*,*) 'NEWCOLORRGB table ic=',ic,' ',irgb,' ',
c     &             COLOR_RGB(ic),
c     &             COLOR_NAME(ic),' ci ',G_COLOR_CINDEX(ic)
        if(irgb.eq.COLOR_RGB(ic)) then
          call NEWCOLOR(ic)
          return  
        endif
      end do
c
      N = N_COLOR + 1
      if(N.gt.Ncolors_max) then
        write(*,*)
     &      'NEWCOLORRGB: Colortable overflow. New color ignored.'
        return
      endif
      G_COLOR_CINDEX(N) = -1
      COLOR_RGB(N)  = iblue + 256*(igreen + 256*ired)
      COLOR_NAME(N) = 'RGBCOLOR'
      N_COLOR = N
      call NEWCOLOR(N)
      return
      end



      subroutine GETCOLORRGB(icol,ired,igrn,iblu,colorname)
C...Gets color information for color designated by index icol
C                 if icol<=0, color -icol in Spectrum is queried
C   Returns   ired    - red   color component (0-255)   (-1 if no color)
C             igrn    - green color component (0-255)   (-1 if no color)
C             iblu    - blue  color component (0-255)   (-1 if no color)
C             colorname - name of current color (lowercase)
C
      include 'pltlib.inc'
      character*(*) colorname
C
C...First assume color "icol" does not exist
      ired = -1
      igrn = -1
      iblu = -1
      colorname = ' '
c
      if(icol.GT.0) then
        ic = icol
      else
        if(-icol.GT.N_SPECTRUM) then
          write(*,*) 'GETCOLORRGB: spectrum index out of bounds: ',
     &                -icol,N_SPECTRUM
          return
        endif
        ic = IFIRST_SPECTRUM - icol - 1
      endif
c
      if(ic.GT.N_COLOR) then
          write(*,*) 'GETCOLORRGB: color index out of bounds: ',
     &                ic,N_COLOR
        return
      endif
c
      irgb = COLOR_RGB(ic)
      irg = irgb/256 
      ired = irg/256 
      igrn = irg  - 256*ired
      iblu = irgb - 256*irg
      colorname = COLOR_NAME(ic)
c
      return
      end


      subroutine convrt2uc(input)
C...Convert string to uppercase
C   Note that the string must be writeable (a variable, not a constant)
c
      character*(*) input
      character*26 lcase, ucase
      data lcase /'abcdefghijklmnopqrstuvwxyz'/
      data ucase /'ABCDEFGHIJKLMNOPQRSTUVWXYZ'/
c
      n = len(input)
      do i=1, n
        k = index(lcase, input(i:i))
        if(k.gt.0) input(i:i) = ucase(k:k)
      end do
c
      return
      end 


      subroutine GETNUMCOLOR(ncol)
C...Gets current number of defined colors
C
      include 'pltlib.inc'
      ncol = N_COLOR
      return
      end


      subroutine GETNUMSPECTRUM(nspec)
C...Gets current number of defined colors in Spectrum
C
      include 'pltlib.inc'
      nspec = N_SPECTRUM
      return
      end


      subroutine COLORMAPDEFAULT
C
C...Creates default colormap palette containing a small number of basic 
C   colors defined in DATA statement below.  The first two colors 
C   are used as the default foreground and background.
C
C   The default colormap contains these defined colors
C        BLACK   =  1
C        WHITE   =  2
C        YELLOW  =  3
C        ORANGE  =  4
C        RED     =  5
C        GREEN   =  6
C        CYAN    =  7
C        BLUE    =  8
C        MAGENTA =  9
C        VIOLET  =  10
C
C   These colors are then accessible by a normal NEWCOLOR(icol) call:
C       icol = 1 .. NCMAP
C
C   Also installs the RGB components of these colors and these color
C   names in the color table.  The colorindex is set to -1 to indicate 
C   that the color has not yet been mapped to the screen color hardware 
C   (this step happens the first time the color is actually used).
C
      include 'pltlib.inc'
c
      PARAMETER (NCMAP=10)
C
      INTEGER       DEFCMAPRGB1(3,NCMAP), DEFCMAPRGB0(3,NCMAP)
      CHARACTER*10  DEFCMAPNAMES(NCMAP)
c
      SAVE ifirst
      DATA ifirst / 0 /
c
c      DATA  ((DEFCMAPRGB(L,I),L=1,3),I=1,NCMAP) 
c     &              /    0,   0,   0,   ! black  
c     &                 255, 255, 255,   ! white  
c     &                 255,   0,   0,   ! red    
c     &                 255, 165,   0,   ! orange 
c     &                 255, 255,   0,   ! yellow 
c     &                   0, 255,   0,   ! green  
c     &                   0, 255, 255,   ! cyan   
c     &                   0,   0, 255,   ! blue   
c     &                 148,   0, 211 /  ! violet 
c     &                 255,   0, 255,   ! magenta
C
C---- hues for reverse-video (black background), use full saturation
      DATA  ((DEFCMAPRGB1(L,I),L=1,3),I=1,NCMAP) 
     &              /    0,   0,   0,   ! black  
     &                 255, 255, 255,   ! white  
     &                 255,   0,   0,   ! red    
     &                 255, 165,   0,   ! orange 
     &                 255, 255,   0,   ! yellow 
     &                   0, 255,   0,   ! green  
     &                   0, 255, 255,   ! cyan   
     &                  30, 140, 255,   ! blue   
     &                 205,  55, 255,   ! violet 
     &                 255,   0, 255 /  ! magenta
C
C---- hues for regular-video (white background), use partial saturation
      DATA  ((DEFCMAPRGB0(L,I),L=1,3),I=1,NCMAP) 
     &              /    0,   0,   0,   ! black  
     &                 255, 255, 255,   ! white  
     &                 255,   0,   0,   ! red    
     &                 255, 165,   0,   ! orange 
     &                 220, 220,   0,   ! yellow 
     &                   0, 225,   0,   ! green  
     &                   0, 210, 210,   ! cyan   
     &                  30, 140, 255,   ! blue   
     &                 205,  55, 255,   ! violet 
     &                 255,   0, 255 /  ! magenta
C
       DATA  DEFCMAPNAMES
     &               / 'BLACK     ',
     &                 'WHITE     ',
     &                 'RED       ',
     &                 'ORANGE    ',
     &                 'YELLOW    ',
     &                 'GREEN     ',
     &                 'CYAN      ',
     &                 'BLUE      ',
     &                 'VIOLET    ',
     &                 'MAGENTA   ' /
C
c 
C---Initialize the colormap indices for first call
      if(ifirst.EQ.0) then
        N_COLOR = 0
        N_SPECTRUM = 0
        IFIRST_SPECTRUM = 0
        ifirst = 1
      endif
C
C--- Skip installing new default map if there already are NCMAP colors
      if(N_COLOR.EQ.NCMAP) return
C
C--- Flush current colormap if any, to free up space for new map
      if(N_COLOR.GT.0) call gw_newcmap
c
C--- Fill in the colormap with with the default colors and set colorindex 
C    to -1 to indicate that the color is still unallocated by hardware
      IF(LGW_REVVIDEO) THEN
        do n = 1, NCMAP
          ired = DEFCMAPRGB1(1,n)
          igrn = DEFCMAPRGB1(2,n)
          iblu = DEFCMAPRGB1(3,n)
          COLOR_RGB(n)  = iblu + 256*(igrn + 256*ired)
          COLOR_NAME(n) = DEFCMAPNAMES(n)
          G_COLOR_CINDEX(n) = -1
        end do
      ELSE
        do n = 1, NCMAP
          ired = DEFCMAPRGB0(1,n)
          igrn = DEFCMAPRGB0(2,n)
          iblu = DEFCMAPRGB0(3,n)
          COLOR_RGB(n)  = iblu + 256*(igrn + 256*ired)
          COLOR_NAME(n) = DEFCMAPNAMES(n)
          G_COLOR_CINDEX(n) = -1
        end do
      ENDIF
C
      N_COLOR = NCMAP
c     write(*,*) 'COLORMAPDEFAULT: NCOLOR ',N_COLOR
C
      return
      end


      subroutine COLORSPECTRUMHUES(ncols,HUESTR)
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
      include 'pltlib.inc'
c
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
     &      0,   240,   240,  1.0,    ! Cyan
     &     32,   255,    32,  1.0,    ! Green
     &    240,   240,     0,  1.0,    ! Yellow
     &    255,   160,     0,  1.0,    ! Orange
     &    255,    32,    32,  1.5  /  ! Red
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



      subroutine COLORSPECTRUMTRP(ncols,NBASE,IRGBBASE,COLWIDTH)
C...Interpolates a color "Spectrum" table of 1..ncols colors that are 
C   a continuous blend between a small number of defined base colors.
C   The blending between the base colors is controlled by the color 
C   "width" COLWIDTH.  
C
C  Input:
C     ncols    number desired interpolated colors in spectrum
C     NBASE    number base colors defined in IRGBBASE
C     IRGBBASE array(3,*) of integer RGB components for the base colors
C     COLWIDTH color pseudo "width" to use for interpolation
C
C   Overwrites the definition of any existing Spectrum.
C
C
      DIMENSION IRGBBASE(3,NBASE)
      DIMENSION COLWIDTH(NBASE)
C
      include 'pltlib.inc'
C
      DIMENSION COLAXIS(NColors_max), IRGBTBL(3,NColors_max)
c
      if(NBASE.GT.NColors_max)
     &  STOP 'COLORSPECTRUM: Local IRGBBASE array overflow.'
C
C
C---Don't allow less than 2 spectrum colors defined by interpolation table
      if(ncols.LT.2) return
c
C--- Check to make sure we have enough room in the color table
      if(N_COLOR+ncols+1 .gt. Ncolors_max) then
        write(*,*) 'COLORSPECTRUMTRP: Too many colors specified.'
        return
      endif
C
      COLAXIS(1) = 0.
      do ibase=2, NBASE
        COLAXIS(ibase) = COLAXIS(ibase-1)
     &                + 0.5*(COLWIDTH(ibase-1)+COLWIDTH(ibase))
        if(COLAXIS(ibase) .LE. COLAXIS(ibase-1))
     &   STOP 'COLORSPECTRUM: Non-monotonic color axis. Check COLWIDTH.'
      enddo
C
C--- Now fill in the rgb table for the Spectrum colors, 
C    interpolating colors between the entries in the passed-in color table
      ibase = 1
      do i = 1, ncols
        xcol = COLAXIS(NBASE) * float(i-1)/float(ncols-1)
c
 5      xnorm = (xcol            -COLAXIS(ibase))
     &        / (COLAXIS(ibase+1)-COLAXIS(ibase))
c
        if(xnorm.GT.1.0  .AND.  ibase.LT.NBASE) then
          ibase = ibase + 1
          go to 5
        endif
c
        w0 = COLWIDTH(ibase  )
        w1 = COLWIDTH(ibase+1)
        frac = w1*xnorm / (w0 + (w1-w0)*xnorm)
C
        red0 = float(IRGBBASE(1,ibase)  )
        grn0 = float(IRGBBASE(2,ibase)  )
        blu0 = float(IRGBBASE(3,ibase)  )
        red1 = float(IRGBBASE(1,ibase+1))
        grn1 = float(IRGBBASE(2,ibase+1))
        blu1 = float(IRGBBASE(3,ibase+1))
c
        IRGBTBL(1,i) = ifix( (red0 + frac*(red1-red0)) + 0.5 )
        IRGBTBL(2,i) = ifix( (grn0 + frac*(grn1-grn0)) + 0.5 )
        IRGBTBL(3,i) = ifix( (blu0 + frac*(blu1-blu0)) + 0.5 )
      end do
      call COLORSPECTRUMRGB(ncols,IRGBTBL)
c
      return
      end


      subroutine COLORSPECTRUMRGB(NRGB,IRGB)
C...Sets up a color "Spectrum" table for NRGB colors that are 
C   defined by r,g,b values (0-255) in the IRGB array. 
C
C Input:
C     NRGB    number r,g,b colors defined in IRGB
C     IRGB    array(3,*) of integer RGB components for the colors
C
C   Overwrites any existing Spectrum.
C
      DIMENSION IRGB(3,NRGB)
C
      include 'pltlib.inc'
C
      if(N_COLOR.LE.0 .OR. N_COLOR.GT.10) then
        CALL COLORMAPDEFAULT
      endif
C
C--- Check to make sure we have enough room in the color table
      if(N_COLOR+NRGB .gt. Ncolors_max) then
        write(*,*) 'COLORSPECTRUMRGB: Too many colors specified.'
        return
      endif
C
C--- starting index of Spectrum in colormap arrays
      IFIRST_SPECTRUM = N_COLOR + 1
C
C--- Now fill in the Spectrum colors from the passed-in color table
      do i = 1, NRGB
        ired = IRGB(1,i)
        igrn = IRGB(2,i)
        iblu = IRGB(3,i)
C
        IC = IFIRST_SPECTRUM + i - 1
c
        COLOR_RGB(IC)  = iblu + 256*(igrn + 256*ired)
        COLOR_NAME(IC) = 'SPECTRUM'
        G_COLOR_CINDEX(IC) = -1
      end do
c
      N_SPECTRUM = NRGB
      N_COLOR = IC
c     write(*,*) 'COLORSPECTRUMRGB: NCOLOR,NSPECTRUM ',N_COLOR,N_SPECTRUM
c
      return
      end



      subroutine LWR2UPR(INPUT)
      CHARACTER*(*) INPUT
C
      CHARACTER*26 LCASE, UCASE
      DATA LCASE / 'abcdefghijklmnopqrstuvwxyz' /
      DATA UCASE / 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' /
C
      N = LEN(INPUT)
C
      do I=1, N
        K = INDEX( LCASE , INPUT(I:I) )
        IF(K.GT.0) INPUT(I:I) = UCASE(K:K)
      end do
C
      return
      end 






