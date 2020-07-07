C***********************************************************************
C    Module:  ps_subs.f
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
C --- Xplot11 driver for postscript output to file
C
C     Version 4.46 11/28/01
C
C     Notes:  PS Plotting coordinates in points (1pt=1/72in.)
C             are multiplied by 10 and truncated as integers to 
C             eliminate extra characters (decimal pts.) in plot file.
C             Coordinates are converted to points again by output macros.
C
C             Landscape orientation is done by translation and rotation 
C             of upright plot.  
C
C             The option exists to check before overwriting all old 
C             postscript output files (including the default file 
C             "plot.ps") see the commented section below. 
C***********************************************************************

      subroutine ps_setup(nunit)
C
C---Sets defaults for Postscript output
C
C    nunit   specifies logical unit and suffix for name for .ps output file
C            if nunit<0 output file is named "plotXXX.ps" where XXX is the
C               plot sequential number (i.e. separate plot files are created 
C               for each plot) 
C            if nunit=0 output file is named "plot.ps"  
C            if nunit>0  output file is named "plotUUU.ps" where UUU is the 
C               nunit unit number
C
      include 'pltlib.inc'
      character numunit*3
C
      PX_ORG = 10.
      PY_ORG = 10.
      P_SCALE = 0.
      LPS_OPEN      = .FALSE. 
      LPS_UNSTROKED = .FALSE.
      LPS_EXTERNAL  = .FALSE.
      LPS_ONEFILE   = .TRUE.
      I_PAGES = 0
C
C---Default postscript output file is "plot.ps", specified if nunit=0,
C   with logical IO to Fortran unit NPS_UNIT_DEFAULT.
C   (Note that unit NPRIM_UNIT_DEFAULT is also reserved and is dedicated
C    to the primitives overflow file)
C
C   If user specifies nunit<0 each plot will be printed to a separate
C   file with name assigned as "plotNNN.ps" where NNN is the sequential 
C   plot number
C
C   If user specifies a logical unit to use for the plot file the logical
C   unit is used for IO and the name assigned is "plotNNN.ps" where
C   NNN is the logical unit number (0>NNN<1000)
C
      nunit0 = nunit
      if(nunit.EQ.0) then
       nunit0 = NPS_UNIT_DEFAULT
       PS_FILE = 'plot.ps'
      elseif(nunit.EQ.NPRIM_UNIT_DEFAULT .OR. nunit.GT.999) then
       write(*,*) 'PS_SETUP: PS file unit out of bounds: ',nunit
       write(*,*) '          Using default unit ',NPS_UNIT_DEFAULT
       write(*,*) '          Using default file "plot.ps"'
       nunit0 = NPS_UNIT_DEFAULT
       PS_FILE = 'plot.ps'
      elseif(nunit.LT.0) then
c       write(*,*) 'PS_SETUP: separate PS files used for each plot'
c       write(*,*) '          Using unit ',NPS_UNIT_DEFAULT
c       write(*,*) '          Using file "plot###.ps"'
       nunit0 = NPS_UNIT_DEFAULT
       PS_FILE = 'plot000.ps'
       LPS_ONEFILE = .FALSE.
      else
       write(numunit,10) nunit0
       PS_FILE = 'plotunit' // numunit // '.ps'
      endif
      NPS_UNIT = nunit0 
C
 10   format(I3.3)
      return
      end


      subroutine ps_init
C---Initializes Postscript plotting and global plot variables
      include 'pltlib.inc'
C
C---Change page orientation if required
      IPS_MODE = 0
      if(I_PAGETYPE.EQ.Page_Landscape) IPS_MODE = 1
C
      N_VECS  = 0
C...P_SCALE set so user graphics scales to 1.0inch/(absolute unit) on page
      if(P_SCALE.EQ.0.)  P_SCALE = 72.
      PX_SIZ = P_SCALE*X_PAGE
      PY_SIZ = P_SCALE*Y_PAGE
      call ps_open
C
      return
      end


      subroutine ps_open
C...Initializes PostScript file for plotting commands
      include 'pltlib.inc'
      logical LEXIST, LOPEN
      character*80 PS_FILE2
      character*1 ans
      character numpage*3
C
      LOPEN = LPS_OPEN
      call a_strip(' ',PS_FILE)
      NCH = index(PS_FILE,' ') - 1
C
C---Check status on PS file
C---PS file NOT OPENED.
      if(.NOT.LPS_OPEN) then
C
C---Check unit to see if user opened it already (set status flag LPS_EXTERNAL)
C   if file is already open skip opening the file, just use it
C   if unit is unopened then open a file for output
        if(LPS_EXTERNAL) go to 10
C---Check if file pre-opened outside of Xplot11
        inquire(unit=NPS_UNIT,opened=LPS_EXTERNAL,err=1)
C
 1      if(.NOT.LPS_EXTERNAL) then
C
C---If we are writing separate plotxxx.ps files create plot file name for
C   this plot using cumulative plot number
         if(.NOT.LPS_ONEFILE) then
           write(numpage,100) N_PAGES
           PS_FILE = 'plot' // numpage // '.ps'
         endif
 100     format(I3.3)
C
C
C.....PS_FILE doesn't exist, so open it and proceed
 2       open(unit=NPS_UNIT,file=PS_FILE,status='UNKNOWN',err=3)
         rewind(NPS_UNIT)
         go to 10
C....On open error get some other name and try again...
 3       write(*,1020)
         read (*,1100) PS_FILE2
         call a_strip(' ',PS_FILE2)
         if(PS_FILE2.EQ.' ') go to 3
         PS_FILE = PS_FILE2
         NCH = index(PS_FILE,' ') - 1
         go to 2
C
        endif
C
 1010   format(/' PostScript output file  ',A,
     &          '  exists. Overwrite? [Y] ',$)
 1020   format(' Specify new output file: ',$)
C
C...Write Postscript file header to identify this as a .ps file
 10     write(NPS_UNIT,1030) PS_FILE
        if(IPS_MODE.EQ.1) write(NPS_UNIT,1040) 
        write(NPS_UNIT,1050) 
C
 1030   format('%!PS-Adobe-2.0'/
     &         '%%Title: ',A/
     &         '%%Creator: Xplot11'/
     &         '%%Pages: (atend)'/
     &         '%%BoundingBox: (atend)')
 1040   format('%%Orientation: Landscape')
 1050   format('%%EndComments'/)
C
        LPS_OPEN = .TRUE.
C
      endif
C
C
C...For any ps_open -> Initialize Postscript last point and bounding box
      PS_LSTX  = -99999.
      PS_LSTY  = -99999.
      BB_XMIN  =  99999.
      BB_YMIN  =  99999.
      BB_XMAX  = -99999.
      BB_YMAX  = -99999.
C
      if(LOPEN) then
         if(LPS_EXTERNAL) then
           write(*,1064)
          else
           write(*,1065) PS_FILE(1:NCH)
         endif
       else
         if(LPS_EXTERNAL) then
           write(*,1059)
          else
           write(*,1060) PS_FILE(1:NCH)
         endif
      endif
 1059 format(' Writing   PostScript to external file ...')
 1060 format(' Writing   PostScript to file  ',A,' ...')
 1064 format(' Appending PostScript to external file ...')
 1065 format(' Appending PostScript to file  ',A,' ...')
C
C...Put out a page preamble
      N_PAGES = N_PAGES + 1
      I_PAGES = I_PAGES + 1
      write(NPS_UNIT,1070) I_PAGES, I_PAGES
 1070 format('%%Page: ',I4,2X,I4/
     &       'gsave  %Save current context'/
     &       '% Define macros for drawing'/
     &       '/rscal {10 div exch 10 div exch } bind def'/
     &       '/M { rscal moveto } bind def'/
     &       '/L { rscal lineto } bind def'/
     &       '/SG { setgray } bind def'/
     &       '/NP { newpath } bind def'/
     &       '/SL { setlinewidth } bind def'/
     &       '/CPSM { currentpoint stroke moveto } bind def'/
     &       '/CFS  { closepath fill stroke } bind def'/
     &       '/CO { 2 index 255 div 2 index 255 div 2 index 255 div'/ 
     &       '      setrgbcolor pop pop pop } bind def'/
     &       '/LAND { 0 790 translate -90 rotate } bind def'//
     &       '% Set up for default line type and width'/
     &       ' 1 setlinejoin 0.25 SL [ ] 0 setdash 0 SG')
C
C---Use one of these sets for rotating/translating to landscape, depending 
C   on your postscript screen viewer (right-side up/upside-down)
C--------------------------------------------------------------
c      ixtrans =  0
c      iytrans =  ifix(P_SCALE*X_PAGE)
c      irotate = -90
C--------------------------------------------------------------
      ixtrans =  ifix(P_SCALE*Y_PAGE)
      iytrans =  0
      irotate =  90
C--------------------------------------------------------------
 1080 format('% Rotate and translate for Landscape format'/
     &       I4,1X,I4,' translate ',I4,' rotate')
      if(IPS_MODE.EQ.1) write(NPS_UNIT,1080) ixtrans,iytrans,irotate
C
      LPS_UNSTROKED = .TRUE.
      N_VECS = 0
C
 1100 format(a)
      return
      end


      subroutine a_strip(ALPH,STRING)
C
C---- Strips all leading ALPH characters from STRING
      character*(*) STRING
      character*1 ALPH
c
      num = len(STRING)
      do k=1, num
        if(INDEX(STRING(k:k),ALPH) .EQ. 0) go to 10
      enddo
      return
c
 10   STRING = STRING(k:num)
      return
      end


      subroutine ps_close
C...Closes PostScript file for plotting
      include 'pltlib.inc'
C
      if(.NOT.LPS_OPEN) return
C
C...Put out page count
      write(NPS_UNIT,30) I_PAGES
  30  format('%%Trailer'/'%%Pages: ',I4)
C
C...Don't mess with external supplied units
      if(.NOT.LPS_EXTERNAL) then
        close(NPS_UNIT)
        NPS_UNIT = -1
      endif
C
      LPS_OPEN = .FALSE.
      return
      end


      subroutine ps_endpage
C...Ends PostScript page
      include 'pltlib.inc'
C
      if(.NOT.LPS_OPEN     ) return
      if(.NOT.LPS_UNSTROKED) return
C
C...If a page has already been plotted, finish it
      if(I_PAGES.GT.0) THEN
        write(NPS_UNIT,20) BB_XMIN,BB_YMIN,
     &                     BB_XMAX,BB_YMAX
      endif
  20  format('stroke showpage grestore'/'%%BoundingBox: ',4F8.1/)
C
      LPS_UNSTROKED = .FALSE.
      N_VECS = 0
C
      return
      end


      subroutine ps_flush
C...Flushes out buffered plot output to PostScript file
      include 'pltlib.inc'
      return
      end


      subroutine ps_color(icolor)
C...Sets PostScript foreground color from stored RGB colormap
C   Note: The background color for PS is always white
C         the foreground color is normally black 
C         you get color when color PS printing is enabled 
C         and the color is set to one of the colors in the color tables
C    icolor =  1 mapped to black
C    icolor =  2 mapped to white
C     ...
C    icolor =  N_COLOR mapped to last color in color table
C   See the colormapping routines in plt_color.f for assigned colors
C
      include 'pltlib.inc'
      character*22 colorname
C
      if(.NOT.LPS_OPEN .OR. .NOT.LPS_COLOR) return
C
C...Flush out existing lines at old color
      if(N_VECS.GT.0) then
        write(NPS_UNIT,10)
        N_VECS = 0
      endif
C
C---Consult color map for RGB values 
      icol = icolor
      if(N_COLOR.LE.0) icol = 1
      call GETCOLORRGB(icol,ired,igrn,iblu,colorname)
C
C---RGB goes directly into postscript as color spec
      write(NPS_UNIT,20) ired,igrn,iblu
C
  10  format(' CPSM')
  20  format(' ',3(I5),' CO')
      return
      end

      subroutine ps_pen(jpen)
C...Sets PostScript line width
      include 'pltlib.inc'
C
      if(.NOT.LPS_OPEN) return
C
C...Change the line width for new lines
      if(N_VECS.GT.0) then
        write(NPS_UNIT,10)
        N_VECS = 0
      endif
C
      write(NPS_UNIT,20) 0.25*float(jpen)
C
  10  format(' CPSM')
  20  format(' ',F5.2,' SL')
      return
      end

      subroutine ps_linepattern(lmask)
C...Sets Postscript line pattern 
      include 'pltlib.inc'
C
      dimension iseg(32)
      data mskall /-1/
      data nsegmax / 8 /
C
      if(.NOT.LPS_OPEN) return
C
      if(lmask.EQ.0 .OR. lmask.eq.mskall) then
        if(N_VECS.GT.0) write(NPS_UNIT,10) 
        write(NPS_UNIT,20) 
C
       else
C...Set line pattern from lower 16 bits of line mask (integer)
C   Note: no more than 10 pattern elements can be written to PS!
        call bitpat(lmask,nseg,iseg)
        nsg = min(nseg,nsegmax)
	if(N_VECS.GT.0) write(NPS_UNIT,10)
	write(NPS_UNIT,30) (iseg(i),i=1,nsg)
	write(NPS_UNIT,40)
      endif
C
      N_VECS = 0
  10  format(' CPSM')
  20  format(' [ ] 0 setdash')
  30  format(' [',10I3)
  40  format(' ] 0 setdash')
C
      return
      end


      subroutine ps_line(X1,Y1,X2,Y2)
C
C...Plots vector in absolute coordinates to PostScript file
C
C   Note: coordinates are multiplied by 10 and truncated to integers (now
C         accurate to 1/10 of a point, or 1/720 in) to reduce the size of 
C         the ascii plot file.  Note that the moveto and lineto commands
C         defined in the preamble divide these by 10 before they hit the 
C         paper.
C
      include 'pltlib.inc'
C
      if(.NOT.LPS_OPEN) return
C
      PX1 = X1*P_SCALE + PX_ORG
      PY1 = Y1*P_SCALE + PY_ORG
      PX2 = X2*P_SCALE + PX_ORG
      PY2 = Y2*P_SCALE + PY_ORG
      BB_XMAX = MAX(BB_XMAX,PX1,PX2)
      BB_XMIN = MIN(BB_XMIN,PX1,PX2)
      BB_YMAX = MAX(BB_YMAX,PY1,PY2)
      BB_YMIN = MIN(BB_YMIN,PY1,PY2)
      ipx1 = ifix(10.0*PX1)
      ipy1 = ifix(10.0*PY1)
      ipx2 = ifix(10.0*PX2)
      ipy2 = ifix(10.0*PY2)
C
      if(N_VECS.GE.500) then
	write(NPS_UNIT,10)
	N_VECS = 0
      endif
C
      if(PX1.EQ.PS_LSTX .AND. PY1.EQ.PS_LSTY .AND. N_VECS.NE.0) then
        write(NPS_UNIT,30) ipx2,ipy2
       else
	write(NPS_UNIT,20) ipx1,ipy1,ipx2,ipy2
      endif
C
      PS_LSTX = PX2
      PS_LSTY = PY2
      N_VECS = N_VECS + 1
C
  10  format(' CPSM')
  20  format(i5,1x,i5,' M ',i5,1x,i5,' L')
  30  format(i5,1x,i5,' L')
C
      return
      end


      subroutine ps_setscale(factor)
C---Resets postscript plot scaling to factor*72pts/in
      include 'pltlib.inc'
C...P_SCALE set so user graphics scales to factor of 1.0inch/(absolute unit)
      P_SCALE = factor*72.
      PX_SIZ = P_SCALE*X_PAGE
      PY_SIZ = P_SCALE*Y_PAGE
      return
      end



      subroutine ps_polyline(X,Y,n,ifill)
C...Plots polyline to postscript output	
C
C   Note for non-color postscript plots, colors in the colormap spectrum
C   can be used to shade filled polylines with a grey fill spectrum.
C
C   Note: this simply uses the ps_line routine to put up the path,
C         then fills and strokes the path.  It is important that 
C         the number of points not exceed the stroke limit in ps_line
C         or it will try to stroke the path we need to fill...
C
      include 'pltlib.inc'
      real mingrey, maxgrey
      dimension X(n), Y(n)
      data mingrey, maxgrey / 0.10, 0.95 /
      if(n.LE.1) return
C
C...If this is a filled polyline flush out existing lines
      if(N_VECS.GT.0) then
        write(NPS_UNIT,10)
        N_VECS = 0
      endif
C
      X1 = X(1)
      Y1 = Y(1)
      do i = 2, n
        X2 = X(I)
        Y2 = Y(i)
        call ps_line(X1,Y1,X2,Y2)
        X1 = X2
        Y1 = Y2
      end do
C
C...If this is not a color PS plot, shade any Spectrum color indices with
C   a grey shade from light grey to near black to replace the color shading
      if(ifill.eq.0) then 
        write(NPS_UNIT,10)
      else
        grey = 0.0
        if(.NOT.LPS_COLOR .AND. N_COLOR.GT.0) then
          call GETCOLOR(icol)
          if(icol.EQ.2) then
            grey = 1.0
           elseif(icol.LT.0) then
            ispec = -icol
            greyfrac = float(ispec-1)/float(N_SPECTRUM-1)
            grey = mingrey + (maxgrey-mingrey)*greyfrac
           else
            grey = 0.0
          endif
          write(NPS_UNIT,15) grey
        endif
        write(NPS_UNIT,20)
      endif
C...Flush vector count since we are shading this now
      N_VECS = 0
      if(grey.NE.0.0) then
        grey = 0.0
        write(NPS_UNIT,15) grey
      endif
C
  10  format(' CPSM')
  15  format(' ',F5.2,' SG')
  20  format(' CFS')
      return
      end


      subroutine bitpat(mask,nout,iout)
c
c--- Takes an integer mask and returns an integer array which contains
c    the on/off bit pattern, 
c     for example:  a mask with 0001000100010001 returns 8 integers 
c                   in the iout array (1,3,1,3,1,3,1,3)
c                   a mask with 1110111011101110 returns 8 integers 
c                   in the iout array (3,1,3,1,3,1,3,1)
c     Note: the bit mask is shifted to always start counting on a '1' bit.
c
c---  Uses the library routines and() and rshift() for bit manipulation 
c     which are present in most fortran libraries as extensions to f77.
c
      dimension iout(*)
c
c--- Shift the mask until the low order bit is 1 to start...
      imask = mask
      do n = 1, 16
        ibitold = and(1,imask)
        if(ibitold.NE.0) go to 5
          nshft = n
          imask = rshift(imask,1)
      end do
c
 5    nout  = 0
      if(nshft.GE.16) return
      nbits = 0
c
c--- Cycle through 16 shifts to the right looking at the lower bit
c    and comparing with the previous one. If the bit changes record
c    the number of preceding contiguous bits in the output array.
c
      do n = 1, 16-nshft
        ibit = and(1,imask)
ccc        write(*,*) 'n, imask,ibit,ibitold ',n,imask,ibit,ibitold
c
        if(ibit.ne.ibitold) then
          nout = nout + 1
          iout(nout) = nbits
ccc          write(*,*) 'nout ',nout,' adding nbits to iout ',nbits
          nbits = 0
        endif
c
        ibitold = ibit
        nbits = nbits + 1
        imask = rshift(imask,1)
      end do
c--- Add final bit(s) to end of 16 bits checked
c--- Now append any zero bits shifted out originally
      if(ibit.EQ.1) then
        nout = nout + 1
        iout(nout) = nbits
        if(nshft.GT.0) then
          nout = nout + 1
          iout(nout) = nshft
        endif
       else
        nout = nout + 1
        iout(nout) = nbits + nshft
      endif
c
      return
      end






