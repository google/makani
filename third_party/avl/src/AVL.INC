C
      INCLUDE 'AINDEX.INC'
C
C...Parameter and Global variables for Athena 3-D vortex lattice code
C
C   Primary array limits
C
C    NVMAX   number of horseshoe vortices
C    NSMAX   number of chord strips
C    NFMAX   number of surfaces
C
C    NLMAX   number of source/doublet line nodes
C    NBMAX   number of bodies
C
C    NUMAX   number of freestream parameters (V,Omega)
C    NDMAX   number of control deflection parameters
C    NGMAX   number of design variables
C
C    NRMAX   number of stored run cases
C    NTMAX   number of stored time levels
C
      PARAMETER (NVMAX=6000, 
     &           NSMAX=400, 
     &           NFMAX=30,
     &           NLMAX=500,
     &           NBMAX=20,
     &           NUMAX=6,
     &           NDMAX=30,
     &           NGMAX=20,
     &           NRMAX=25,
     &           NTMAX=5000 )
C
C   Derived array limits
C
C    ITMAX  number of triangles for hidden-line routines
C    IPMAX  number of parameters
C    ICMAX  number of constraints
C    JEMAX  number of eigenmode components
C
      PARAMETER (ITMAX=2*NVMAX,
     &           IVMAX=IVTOT+NDMAX,
     &           ICMAX=ICTOT+NDMAX,
     &           IPMAX=IPTOT,
     &           JEMAX=JETOT )
C
C
C---- unit values, names, and namelengths
      CHARACTER*32 UNCHL,UNCHM,UNCHT,UNCHF,UNCHS,UNCHV,UNCHA,UNCHI,UNCHD
      COMMON /UN_R/UNITL,UNITM,UNITT,UNITF,UNITS,UNITV,UNITA,UNITI,UNITD
      COMMON /UN_C/UNCHL,UNCHM,UNCHT,UNCHF,UNCHS,UNCHV,UNCHA,UNCHI,UNCHD
      COMMON /UN_I/NUL  ,NUM  ,NUT  ,NUF  ,NUS  ,NUV  ,NUA  ,NUI  ,NUD
C
      REAL MACH, MACH0
C
      CHARACTER*80 FILDEF, FRNDEF, FMSDEF, FPRDEF, FEVDEF
      CHARACTER*80 TITLE
      CHARACTER*40 STITLE, BTITLE, RTITLE
      CHARACTER*16 DNAME, GNAME
      CHARACTER*12 VARNAM, CONNAM
      CHARACTER*12 VARKEY
      CHARACTER*3  CONKEY
      CHARACTER*10 PARNAM
      CHARACTER*32 PARUNCH
      COMMON /CASE_C/
     & FILDEF,         ! default configuration save file
     & FRNDEF,         ! default run case save file
     & FMSDEF,         ! default mass distribution file
     & FPRDEF,         ! default dimensional parameter file
     & FEVDEF,         ! default eigenvalue save file
     & TITLE,          ! configuration title
     & STITLE(NFMAX),  ! surface title
     & BTITLE(NBMAX),  ! body title
     & RTITLE(NRMAX),  ! run case title
     & DNAME(NDMAX),   ! control variable name
     & GNAME(NGMAX),   ! design  variable name
     & VARNAM(IVMAX),  ! variable   name
     & CONNAM(ICMAX),  ! constraint name
     & VARKEY(IVMAX),  ! variable   selection key
     & CONKEY(ICMAX),  ! constraint selection key
     & PARNAM(IPMAX),  ! run case parameter name
     & PARUNCH(IPMAX)  ! run case parameter unit name
C
      COMMON /CASE_I/
     & LUINP,             ! logical unit for configuration file
     & LURUN,             ! logical unit for run case file
     & LUOUT,             ! logical unit for output dump file
     & LUSTD,             ! logical unit for stability deriv. dump file
     & LUSYS,             ! logical unit for dynamic system matrix dump file
     & IYSYM,IZSYM,       ! y,z image symm.  (0=no image, 1=image)
     & MATSYM,            ! matrix symmetry flag
     & NVOR,              ! number of horseshoe vortices
     & NSTRIP,            ! number of chordwise strips
     & NSURF,             ! number of surfaces
     & NLNODE,            ! number of body source+doublet line nodes
     & NBODY,             ! number of bodies
     & ICON(IVMAX,NRMAX), ! index of constraint for each variable
     & NVTOT,             ! total number of available variables
     & NCTOT,             ! total number of available constraints
     & NPTOT,             ! total number of available parameters
     & NCONTROL,          ! number of control variables
     & NDESIGN,           ! number of design variables
     & NITMAX,            ! max number of Newton iterations
     & IRUN, NRUN,        ! current run case, number of run cases stored
     & IRUNE,             ! target run case for eigenmode calculations
     & IRUNT,             ! target run case for time march initial state
     & ITRIM(NRMAX),      ! trim type used for run case (if any)
     & NEIGEN(NRMAX),     ! number of valid eigenmodes available for run case
     & NEIGENDAT(NRMAX)   ! number reference data eigenvalues
C
      LOGICAL LGEO,LENC,LAIC,LSRD,LVEL,LSOL,LSEN,
     &        LVISC,LMASS,
     &        LCONDEF, LDESDEF,
     &        LPTOT,LPSURF,LPSTRP,LPELE,LPHINGE,LPDERIV,
     &        LBFORCE,
     &        LNASA_SA, LSA_RATES,
     &        LMWAIT
      LOGICAL LPPAR
      COMMON /CASE_L/
     & LGEO,   ! T if geometry exists
     & LENC,   ! T if all normal vectors are valid
     & LAIC,   ! T if AIC matrix has been generated
     & LSRD,   ! T if unit source+doublet strengths are computed
     & LVEL,   ! T if induced velocity matrix has been computed
     & LSOL,   ! T if valid solution exists
     & LSEN,   ! T if valid sensitivities exist
     & LVISC,  ! T if viscous profile drag terms are to be added
     & LMASS,  ! T if mass data has been read in
     & LCONDEF(NDMAX),  ! T if control variable has been declared
     & LDESDEF(NGMAX),  ! T if design  variable has been declared
     & LPTOT,  ! T if total   forces are to be printed
     & LPSURF, ! T if surface forces are to be printed
     & LPSTRP, ! T if strip   forces are to be printed
     & LPELE,  ! T if element forces are to be printed
     & LPHINGE, ! T if hinge moments are to be printed
     & LPDERIV, ! T if stability and control derivs are to be printed
     & LBFORCE, ! T if body forces are to be included in total forces
     & LNASA_SA,    ! T if NASA-std stability axes are to be used
     & LSA_RATES,   ! T if stability-axis rates are to be used
     & LMWAIT,      ! T if mode display is to wait for real time
     & LPPAR(IPMAX)          ! T if parameter value is to be plotted
      COMMON /CASE_R/
     & VERSION,          ! AVL version number
     & DTR,     PI,      ! 3.14159/180 ,  3.14159
     & YSYM,    ZSYM,    ! y- and z-locations of symmetry planes
     & ALFA,   BETA,        ! alpha, beta
     & VINF(3),             ! freestream velocies in body axes
     & VINF_A(3),           ! d(Vinf)/d(alpha)
     & VINF_B(3),           ! d(Vinf)/d(beta)
     & WROT(3),             ! rotation rates in body axes
     & DTIMED,              ! time step for eigenmode movie integration
     & PARVAL(IPMAX,NRMAX),  ! parameter values for run cases
     & CONVAL(ICMAX,NRMAX),  ! imposed constraint value
     & DELCON(NDMAX),        ! imposed control variable value
     & DELDES(NGMAX),        ! imposed design  variable value
     & SREF,  CREF,  BREF,   ! Area, Chord, Span  reference values
     & XYZREF(3),            ! X,Y,Z location for moments
     & XYZREF0(3),           ! X,Y,Z location for moments (default)
     & MACH,                 ! freestream Mach number
     & MACH0,                ! freestream Mach number (default)
     & CDREF,                ! baseline profile CD
     & CDREF0,               ! baseline profile CD (default)
     & VRCORE,               ! vortex core radius / chord
     & SRCORE,               ! source core radius / body radius
     & CLFF, CYFF, CDFF,     ! Trefftz-plane  CL,CY,CDi
     & CLFF_U(NUMAX),CYFF_U(NUMAX),CDFF_U(NUMAX), ! deriv wrt Vinf,Wrot
     & CLFF_D(NDMAX),CYFF_D(NDMAX),CDFF_D(NDMAX), ! deriv wrt control
     & CLFF_G(NGMAX),CYFF_G(NGMAX),CDFF_G(NGMAX), ! deriv wrt design
     & SPANEF,           ! span efficiency
     & SPANEF_A,         ! d(SPANEF)/d(alpha)
     & SPANEF_U(NUMAX),  ! d(SPANEF)/d(beta)
     & SPANEF_D(NDMAX),  ! d(SPANEF)/d(control)
     & SPANEF_G(NGMAX),  ! d(SPANEF)/d(design)
     & CDTOT, CLTOT,          ! total CD,CL
     & CXTOT, CYTOT, CZTOT,   ! total Cx,Cy,Cz
     & CRTOT, CMTOT, CNTOT,   ! total Cl,Cm,Cn
     & CDVTOT,                ! total viscous CD
     & CDTOT_A, CLTOT_A,            ! sensitivities wrt alpha
     & CDTOT_U(NUMAX),CLTOT_U(NUMAX),                !sens wrt U,W
     & CXTOT_U(NUMAX),CYTOT_U(NUMAX),CZTOT_U(NUMAX), !sens wrt U,W
     & CRTOT_U(NUMAX),CMTOT_U(NUMAX),CNTOT_U(NUMAX), !sens wrt U,W
     & CDTOT_D(NDMAX),CLTOT_D(NDMAX),                !sens wrt control
     & CXTOT_D(NDMAX),CYTOT_D(NDMAX),CZTOT_D(NDMAX), !sens wrt control
     & CRTOT_D(NDMAX),CMTOT_D(NDMAX),CNTOT_D(NDMAX), !sens wrt control
     & CDTOT_G(NGMAX),CLTOT_G(NGMAX),                !sens wrt design
     & CXTOT_G(NGMAX),CYTOT_G(NGMAX),CZTOT_G(NGMAX), !sens wrt design
     & CRTOT_G(NGMAX),CMTOT_G(NGMAX),CNTOT_G(NGMAX), !sens wrt design
     & CHINGE(NDMAX), 
     & CHINGE_U(NDMAX,NUMAX),
     & CHINGE_D(NDMAX,NDMAX),
     & CHINGE_G(NDMAX,NGMAX),
     & DCL_A0, DCM_A0,   ! additional default CL_a, CM_a
     & DCL_U0, DCM_U0    ! additional default CL_u, CM_u
      COMPLEX EVAL, EVEC, EVALDAT
      COMMON /CASE_Z/ 
     &  EVAL(JEMAX,NRMAX),       ! mode eigenvalue
     &  EVEC(JEMAX,JEMAX,NRMAX), ! mode eigenvector
     &  EVALDAT(JEMAX,NRMAX)     ! mode eigenvalue reference data

      COMMON /TIME_I/
     &  ITLEV,           ! current time level
     &  NTLEV,           ! number of stored time levels
     &  NTSTEPS          ! default number of time steps to run

      COMMON /TIME_R/
     &  DELTAT,                 ! integration time step
     &  TDER(3),                ! time-derivative differencing coefficient
     &  TLEV(NTMAX),            ! time values
     &  TPARS(KPTOT,NTMAX),     ! scalar parameter time trace
     &  TPARV(3,KPVTOT,NTMAX),  ! vector parameter time trace
     &  TPARD(NDMAX,NTMAX)      ! control parameter time traces

      COMMON /MASS_R/
     &   RHO0, GEE0,     ! density, gravity   | from .mass file
     &   XYZMASS0(3),    ! mass centroid      | from .mass file
     &   RMASS0     ,    ! real mass          | from .mass file
     &   RINER0(3,3),    ! real inertia       | from .mass file
     &   AMASS(3,3),     ! apparent mass/rho     | from geometry
     &   AINER(3,3)      ! apparent inertia/rho  | from geometry


      LOGICAL LFWAKE, LFALBE, LFLOAD
      COMMON /SURF_L/  
     & LFWAKE(NFMAX),   ! T if surface sheds a wake
     & LFALBE(NFMAX),   ! T if surface is to see freestream alpha,beta
     & LFLOAD(NFMAX)    ! T if surface contributes to overall loads

      COMMON /SURF_I/  
     & NJ(NFMAX),       ! number of elements along span  in surface
     & NK(NFMAX),       ! number of elements along chord in surface
     & IFRST(NFMAX),    ! index of first element in surface
     & JFRST(NFMAX),    ! index of first strip in surface
     & IMAGS(NFMAX),    ! indicates whether surface is a YDUPlicated one
     & LSCOMP(NFMAX)    ! logical surface component index

      COMMON /SURF_R/
     & CDSURF(NFMAX),CLSURF(NFMAX),                ! surface CD,CL
     & CXSURF(NFMAX),CYSURF(NFMAX),CZSURF(NFMAX),  ! surface Cx,Cy,Cz
     & CRSURF(NFMAX),CNSURF(NFMAX),CMSURF(NFMAX),  ! surface Cl,Cm,Cn
     & CDVSURF(NFMAX),                             ! surface viscous CD
     & CDS_A(NFMAX), CLS_A(NFMAX),
     & CDS_U(NFMAX,NUMAX), CLS_U(NFMAX,NUMAX),
     & CXS_U(NFMAX,NUMAX), CYS_U(NFMAX,NUMAX), CZS_U(NFMAX,NUMAX),
     & CRS_U(NFMAX,NUMAX), CNS_U(NFMAX,NUMAX), CMS_U(NFMAX,NUMAX),
     & CDS_D(NFMAX,NDMAX), CLS_D(NFMAX,NDMAX),
     & CXS_D(NFMAX,NDMAX), CYS_D(NFMAX,NDMAX), CZS_D(NFMAX,NDMAX),
     & CRS_D(NFMAX,NDMAX), CNS_D(NFMAX,NDMAX), CMS_D(NFMAX,NDMAX),
     & CDS_G(NFMAX,NGMAX), CLS_G(NFMAX,NGMAX),
     & CXS_G(NFMAX,NGMAX), CYS_G(NFMAX,NGMAX), CZS_G(NFMAX,NGMAX),
     & CRS_G(NFMAX,NGMAX), CNS_G(NFMAX,NGMAX), CMS_G(NFMAX,NGMAX),
     & CF_SRF(3,NFMAX), CM_SRF(3,NFMAX), 
     & CL_SRF(NFMAX), CD_SRF(NFMAX), CMLE_SRF(NFMAX),
     & SSURF(NFMAX), CAVESURF(NFMAX),
     & VBODY(NBMAX)
C
C
      COMMON /STRP_I/
     & NSURFS(NSMAX),    ! index of surface which contains this strip
     & IJFRST(NSMAX),    ! index of first element in strip 
     & NVSTRP(NSMAX)     ! number of elements in strip

      LOGICAL LSTRIPOFF,LVISCSTRP,LJ1SECT,LJ2SECT
      COMMON /STRP_L/
     & LSTRIPOFF(NSMAX),  ! T if strip is "turned off" (outside of fluid)
     & LVISCSTRP(NSMAX),  ! T is strip has viscous drag data
     & LJ1SECT(NSMAX),    ! T if station 1 is a section
     & LJ2SECT(NSMAX)     ! T if station 2 is a section

      COMMON /STRP_R/
     & RLE(3,NSMAX),  CHORD(NSMAX),    ! strip c.p. line LE point, chord
     & RLE1(3,NSMAX), CHORD1(NSMAX),   ! strip left  end LE point, chord
     & RLE2(3,NSMAX), CHORD2(NSMAX),   ! strip right end LE point, chord
     & WSTRIP(NSMAX),                  ! strip y-z width
     & TANLE(NSMAX),  TANTE(NSMAX),    ! strip LE,TE sweep slopes
     & CLCD(NUMAX,NSMAX),              ! strip viscous polar
     & SAXFR,                       ! x/c of spanwise axis for Vperp def
     & ESS(3,NSMAX),                ! spanwise unit vector for Vperp def
     & ENSY(NSMAX), ENSZ(NSMAX),    ! strip normal vector in Trefftz-Pln
     & XSREF(NSMAX),YSREF(NSMAX),ZSREF(NSMAX),
     & AINC(NSMAX),              ! strip's incidence twist angle
     & AINC_G(NSMAX,NGMAX),      ! dAINC/dG
     & CDSTRP(NSMAX),   CLSTRP(NSMAX),                   ! strip forces
     & CXSTRP(NSMAX),   CYSTRP(NSMAX),   CZSTRP(NSMAX),  ! strip forces
     & CRSTRP(NSMAX),   CNSTRP(NSMAX),   CMSTRP(NSMAX),  ! strip moments
     & CDST_A(NSMAX),   CLST_A(NSMAX),                   ! alpha sens.
     & CDST_U(NSMAX,NUMAX), CLST_U(NSMAX,NUMAX),
     & CXST_U(NSMAX,NUMAX), CYST_U(NSMAX,NUMAX), CZST_U(NSMAX,NUMAX),
     & CRST_U(NSMAX,NUMAX), CNST_U(NSMAX,NUMAX), CMST_U(NSMAX,NUMAX),
     & CDST_D(NSMAX,NDMAX), CLST_D(NSMAX,NDMAX),
     & CXST_D(NSMAX,NDMAX), CYST_D(NSMAX,NDMAX), CZST_D(NSMAX,NDMAX),
     & CRST_D(NSMAX,NDMAX), CNST_D(NSMAX,NDMAX), CMST_D(NSMAX,NDMAX),
     & CDST_G(NSMAX,NGMAX), CLST_G(NSMAX,NGMAX),
     & CXST_G(NSMAX,NGMAX), CYST_G(NSMAX,NGMAX), CZST_G(NSMAX,NGMAX),
     & CRST_G(NSMAX,NGMAX), CNST_G(NSMAX,NGMAX), CMST_G(NSMAX,NGMAX),
     & CF_STRP(3,NSMAX),   CM_STRP(3,NSMAX), ! strip forces, norm local
     & CNRMSTRP(NSMAX),    CAXLSTRP(NSMAX), 
     & CD_LSTRP(NSMAX),  CL_LSTRP(NSMAX), 
     & CDV_LSTRP(NSMAX), CLTSTRP(NSMAX), CLASTRP(NSMAX),
     & CMC4(NSMAX),        CMLE(NSMAX),
     & CNC(NSMAX),         DWWAKE(NSMAX),
     & CNC_U(NSMAX,NUMAX),
     & CNC_D(NSMAX,NDMAX),
     & CNC_G(NSMAX,NGMAX)
C
      LOGICAL LVNC, LVALBE
      COMMON /VRTX_L/
     & LVNC(NVMAX),    ! T if V.n=0 is to be enforced for this c.p.
     & LVALBE(NVMAX)   ! T if c.p. is to see freestream alpha,beta

      COMMON /VRTX_I/
     & NSURFV(NVMAX)    ! index of surface which contains vortex element
      COMMON /VRTX_R/
     & RV1(3,NVMAX),    ! h.v. vortex left  points
     & RV2(3,NVMAX),    ! h.v. vortex right points
     & RV(3,NVMAX),     ! h.v. vortex center points
     & RC(3,NVMAX),     ! h.v. control points
     & RS(3,NVMAX),     ! h.v. source points
     & RL(3,NLMAX),RADL(NLMAX),  ! source line node points, body radius
     & DXV(NVMAX),               ! chord of element
     & CHORDV(NVMAX),            ! chord of element-containing strip
     & SLOPEV(NVMAX),            ! camber slopes at h.v. bound leg
     & SLOPEC(NVMAX),            ! camber slopes at c.p.
     & DCONTROL(NVMAX,NDMAX),  ! d(normal angle)/dCONTROL
     & VHINGE(3,NSMAX,NDMAX),  ! hinge vector for CONTROL rot. of normal
     & PHINGE(3,NSMAX,NDMAX),  ! point on hingeline for hinge moment calculation
     & VREFL(NSMAX,NDMAX),  !sign applied to hinge vec. of refl. surface
     & ENC(3,NVMAX),           ! c.p. normal vector
     & ENV(3,NVMAX),           ! h.v. normal vector
     & ENC_D(3,NVMAX,NDMAX),   ! sensitivities
     & ENC_G(3,NVMAX,NGMAX),
     & ENV_D(3,NVMAX,NDMAX),
     & ENV_G(3,NVMAX,NGMAX),
     & DCP(NVMAX),             ! delta(Cp) on vortex element
     & DCP_U(NVMAX,NUMAX),
     & DCP_D(NVMAX,NDMAX),
     & DCP_G(NVMAX,NGMAX),
     & GAM(NVMAX),             ! circulation of h.v. vortex
     & GAM_U_0(NVMAX,NUMAX),
     & GAM_U_D(NVMAX,NUMAX,NDMAX),
     & GAM_U_G(NVMAX,NUMAX,NGMAX),
     & GAM_U(NVMAX,NUMAX),
     & GAM_D(NVMAX,NDMAX),
     & GAM_G(NVMAX,NGMAX),
     & SRC(NLMAX),        ! source  strength of source+doublet line elem
     & DBL(3,NLMAX),      ! doublet strength of source+doublet line elem
     & SRC_U(NLMAX,NUMAX),
     & DBL_U(3,NLMAX,NUMAX),
     & WCSRD_U(3,NVMAX,NUMAX),
     & WVSRD_U(3,NVMAX,NUMAX)
C
      COMMON /BODY_I/  
     & NL(NBMAX),       ! number of source-line nodes in body
     & LFRST(NBMAX)     ! index of first line node in body

      COMMON /BODY_R/
     & ELBDY(NBMAX),  ! body length
     & SRFBDY(NBMAX), ! body surface area
     & VOLBDY(NBMAX), ! body volume
     & DCPB(3,NLMAX),                           ! dCp loading on sources
     & CDBDY(NBMAX),CLBDY(NBMAX),               ! body CD,CL
     & CXBDY(NBMAX),CYBDY(NBMAX),CZBDY(NBMAX),  ! body CX,CY,CZ
     & CRBDY(NBMAX),CNBDY(NBMAX),CMBDY(NBMAX)   ! body Cl,Cm,Cn
C
      COMMON /SOLV_I/ 
     & IAPIV(NVMAX)           ! pivot indices for LU solver
      COMMON /SOLV_R/  
     & AMACH,                 ! Mach number at which AIC matrices were computed
     & AICN(NVMAX,NVMAX),     ! normalwash AIC matrix (and VL system matrix)
     & WC_GAM(3,NVMAX,NVMAX), ! c.p. velocity/Gamma influence matrix
     & WV_GAM(3,NVMAX,NVMAX), ! h.v. velocity/Gamma influence matrix
     & WC(3,NVMAX),           ! total induced velocity at c.p.
     & WC_U(3,NVMAX,NUMAX),
     & WC_D(3,NVMAX,NDMAX),
     & WC_G(3,NVMAX,NGMAX),
     & WV(3,NVMAX),           ! total induced velocity at h.v.
     & WV_U(3,NVMAX,NUMAX),
     & WV_D(3,NVMAX,NDMAX),
     & WV_G(3,NVMAX,NGMAX) 
C
      LOGICAL
     & LWAKEPLT,   LLOADPLT,   LHINGEPLT,
     & LBOUNDLEG,  LCHORDLINE, LCAMBER,   LCNTLPTS, 
     & LNRMLPLT,   LAXESPLT,   LRREFPLT,
     & LCLPERPLT,  LDWASHPLT,  LLABSURF,   LTRFORCE,
     & LABEL_SURF, LABEL_STRP, LABEL_VRTX, LABEL_BODY,
     & LPLTNEW,    
     & LPLTSURF,
     & LPLTBODY, 
     & LSVMOV
      COMMON /PLOT_L/          ! plotting flags
     & LWAKEPLT,   LLOADPLT,   LHINGEPLT,
     & LBOUNDLEG,  LCHORDLINE, LCAMBER,   LCNTLPTS, 
     & LNRMLPLT,   LAXESPLT,   LRREFPLT,
     & LCLPERPLT,  LDWASHPLT,  LLABSURF,   LTRFORCE,
     & LABEL_SURF, LABEL_STRP, LABEL_VRTX, LABEL_BODY,
     & LPLTNEW,    
     & LPLTSURF(NFMAX),
     & LPLTBODY(NBMAX),
     & LSVMOV
      COMMON /PLOT_I/ NTRI, IMARKSURF, NAXANN(3), IRCOLOR(NRMAX)
      COMMON /PLOT_R/ TRI(16,ITMAX),
     &                AXMIN(3),AXMAX(3),AXDEL(3),AXSPAN(3),
     &                GMIN(3),GMAX(3),GMINP(3),GMAXP(3),
     &                VMIN(3),VMAX(3),VMINP(3),VMAXP(3),
     &                DTMOVIE,TMOVIE, SLOMOF,
     &                EPHASE, EIGENF, TMOFAC
