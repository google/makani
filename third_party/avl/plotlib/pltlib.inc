C***********************************************************************
C    Module:  pltlib.inc
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
C
C*******************************
C --- Global data for Xplot11
C     Version 4.46 11/28/01
C*******************************
C
      parameter (Nstore_max=100000,Ncolors_max=256,MaxPolyLine=1001)
C
      Real    DEFAULT_PAGEHEIGHT,
     &        DEFAULT_PAGEWIDTH,
     &        DEFAULT_SCREENFRACTION
C
C---Set paper size here
      parameter (DEFAULT_PAGEHEIGHT     = 11.0,   ! Letter size
     &           DEFAULT_PAGEWIDTH      =  8.5,
     &           DEFAULT_SCREENFRACTION =  0.67)
cc      parameter (DEFAULT_PAGEHEIGHT     = 11.65,  ! A4 European
cc     &           DEFAULT_PAGEWIDTH      =  8.25,
cc     &           DEFAULT_SCREENFRACTION =  0.67)
C
      Integer NPS_UNIT_DEFAULT,
     &        NPRIM_UNIT_DEFAULT
C
      parameter (NPS_UNIT_DEFAULT   = 80)
      parameter (NPRIM_UNIT_DEFAULT = 81)
C
      Integer PageCommand,
     &        PlotCommand,
     &        PenCommand, 
     &        ScaleCommand,
     &        PatternCommand,
     &        ColorCommand,
     &        PolylinePointCommand,
     &        PolylineDrawCommand,
     &        MinClipCommand,
     &        MaxClipCommand
C
      parameter (PageCommand          =  1)
      parameter (PlotCommand          =  2)
      parameter (PenCommand           =  3)
      parameter (ScaleCommand         =  4)
      parameter (PatternCommand       =  5)
      parameter (ColorCommand         =  6)
      parameter (PolylinePointCommand =  7)
      parameter (PolylineDrawCommand  =  8)
      parameter (MinClipCommand       =  9)
      parameter (MaxClipCommand       = 10)
C
      Integer Page_Portrait, 
     &        Page_Landscape
C
      parameter (Page_Portrait        = 0)
      parameter (Page_Landscape       = 1)
C
      LOGICAL   LPRIM_OPEN,
     &          LGW_GEN, LGW_OPEN, LGW_COLOR, LGW_REVVIDEO, 
     &          LGW_CHANGED, LGW_RESIZE,
     &          LPS_GEN, LPS_OPEN, LPS_COLOR, 
     &          LPS_UNSTROKED, LPS_EXTERNAL, LPS_ONEFILE
      INTEGER   GX_LOC, GY_LOC, 
     &          GX_ORG, GY_ORG, 
     &          GX_SIZ, GY_SIZ,
     &          GX_CRS, GY_CRS,
     &          G_WIDTH, G_HEIGHT, G_DEPTH,  
     &          G_COLOR_CINDEX
      INTEGER   COLOR_RGB
      CHARACTER COLOR_NAME*22
      CHARACTER PS_FILE*80
C
C...Common storing current plot primitives
      Common /PL_PRIML/
     &       LPRIM_OPEN
      Common /PL_PRIMI/ 
     &       NPRIM_UNIT, 
     &       N_PRIM, N_WRIT, N_INCORE,
     &       I_TYP(Nstore_max), 
     &       I_PRIM(Nstore_max)
      Common /PL_PRIMR/ 
     &       X_PRIM(Nstore_max),
     &       Y_PRIM(Nstore_max)
C
C...Global data for plotting, clipping, etc.
      Common /PL_PLOTI/
     &       I_DEV, I_PAGETYPE,
     &       I_PEN, LST_PEN,
     &       I_PAT, LST_PAT,
     &       I_CLR, LST_CLR
      Common /PL_PLOTR/
     &       X_SCALE, Y_SCALE,
     &       X_ORG, Y_ORG, 
     &       X_LST, Y_LST,
     &       X_PAGE, Y_PAGE,
     &       X_WIND, Y_WIND,
     &       CLP_XMIN, CLP_XMAX, CLP_YMIN, CLP_YMAX,
     &       XOFF_ZOOM,YOFF_ZOOM,
     &       XFAC_ZOOM,YFAC_ZOOM
C
C...Global data for color definition
      Common /PL_COLORI/
     &       N_COLOR, N_SPECTRUM, IFIRST_SPECTRUM,
     &       COLOR_RGB(Ncolors_max)
      Common /PL_COLORC/
     &       COLOR_NAME(Ncolors_max)
C
C...Global data for X-window output
      Common /PL_XWINL/
     &       LGW_GEN, LGW_OPEN, LGW_COLOR, LGW_REVVIDEO, 
     &       LGW_CHANGED, LGW_RESIZE
      Common /PL_XWINI/
     &       GX_LOC, GY_LOC,
     &       GX_ORG, GY_ORG,
     &       GX_SIZ, GY_SIZ,
     &       GX_CRS, GY_CRS,
     &       G_WIDTH, G_HEIGHT, G_DEPTH,
     &       G_COLOR_CINDEX(Ncolors_max)
      Common /PL_XWINR/
     &       G_SCALE, G_SCRNFRAC
C
C...Global data for PostScript output
      Common /PL_PSL/
     &       LPS_GEN, LPS_OPEN, LPS_COLOR, LPS_UNSTROKED, 
     &       LPS_EXTERNAL, LPS_ONEFILE
      Common /PL_PSI/
     &       IPS_MODE, NPS_UNIT, N_PAGES, I_PAGES, N_VECS
      Common /PL_PSR/
     &       PX_ORG,  PY_ORG, PX_SIZ, PY_SIZ, P_SCALE,
     &       BB_XMIN, BB_XMAX, BB_YMIN, BB_YMAX,
     &       PS_LSTX, PS_LSTY
      Common /PL_PSC/
     &       PS_FILE


C
C   LPRIM_OPEN         T if primitives file is open
C   NPRIM_UNIT         Fortran logical unit for primitives file (unit# 0)
C   N_PRIM             Number of accumulated primitives
C   N_WRIT             Number of primitives written to logfile
C   N_INCORE           Number of primitives in buffer array
C   I_TYP(.)           type-of-primitive index
C   I_PRIM(.)          primitive data
C   X_PRIM(.)              "
C   Y_PRIM(.)              "
C
C   I_DEV              "Device" index, specifies X-graphics, PostScript output
C   I_PAGETYPE        = Page_Landscape or Page_Portrait
C   I_PEN, LST_PEN      current and previous pen thickness
C   I_PAT, LST_PAT      current and previous pen pattern
C   I_CLR, LST_CLR      current and previous color
C
C   X_SCALE,Y_SCALE     absolute-unit/user-unit scale factors
C   X_ORG, Y_ORG        user x,y-axis origin in absolute coordinates
C   X_LST, Y_LST        last pen location  in absolute coordinates
C
C   X_PAGE, Y_PAGE      plot page size in absolute units (typ. 11.0"x8.5")
C   X_WIND, Y_WIND      X-window size in absolute units
C                         (will not exceed X_PAGE,Y_PAGE upon window resizing)
C
C   CLP_XMIN, CLP_XMAX  user-specified clipping limits
C   CLP_YMIN, CLP_YMAX  (in absolute coordinates)
C
C   XOFF_ZOOM           zoom offsets
C   YOFF_ZOOM
C   XFAC_ZOOM           zoom scaling factors
C   YFAC_ZOOM
C
C   N_COLOR             total number of defined colors        1..i..N_COLORS
C   N_SPECTRUM          number of defined colors in Spectrum  0..s..N_SPECTRUM
C   IFIRST_SPECTRUM     gives index i of first Spectrum color
C   COLOR_RGB(i)        rgb color index
C   COLOR_NAME(i)       color name string
C
C   LGW_GEN             T if X-window graphics are to be generated
C   LGW_OPEN            T is X-window is currently open
C   LGW_COLOR           T if X-window has color capability
C   LGW_RESIZE          T if X-window is to be resized 
C   LGW_REVVIDEO        T if X-window is in reverse-video (swap white,black)
C
C   GX_LOC, GY_LOC      location of upper-left window corner in screen pixels
C   GX_ORG, GY_ORG      user-specified origin in window pixels
C   GX_SIZ, GY_SIZ      window size in pixels
C   GX_CRS, GY_CRS      user-specified cursor location
C
C   G_SCALE             # pixels / user plot unit
C   G_SCRNFRAC          fraction of screen taken up by X-window on opening
C   G_WIDTH             pixel width  of root X window display
C   G_HEIGHT            pixel height of root X window display
C   G_DEPTH             color bit depth of screen
C   G_COLOR_CINDEX(i)   X-colormap index of Xplot11 color i
C
C   LPS_GEN             T if PostScript output is to be generated
C   LPS_OPEN            T is PostScript file is currently open
C   LPS_COLOR           T if color PostScript is to be generated
C   LPS_UNSTROKED       T if current PostScript page has not been stroked
C   LPS_EXTERNAL        T if postscript file opened/closed outside of Xplot11
C   LPS_ONEFILE         T if one postscript file used for each plot, otherwise
C                         separate numbered plot files will be used for each
C                         plot
C
C   IPS_MODE            0 if Portrait PostScript, 1 if Lansdcape PostScript
C
C   PX_ORG, PY_ORG      location of user-specified origin (in points)
C   PX_SIZ, PY_SIZ      size of PostScript page (in points)
C   P_SCALE             # points / user plot unit
C
C   NPS_UNIT            Fortran logical unit of PostScript file
C   N_PAGES             Cumulative count of PS pages generated 
C   I_PAGES             Number of PS pages generated for current plot file
C   N_VECS              Number of PS vectors generated so far
C
C   BB_XMIN, BB_XMAX    PostScript Bounding Box limits
C   BB_YMIN, BB_YMAX
C
C   PS_LSTX, PS_LSTY    last PostScript move-to or line-to location
C   PS_FILE             PostScript output filename







