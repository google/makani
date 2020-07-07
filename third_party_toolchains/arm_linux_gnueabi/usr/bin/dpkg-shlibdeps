#!/usr/bin/perl
#
# dpkg-shlibdeps
#
# Copyright © 1996 Ian Jackson
# Copyright © 2000 Wichert Akkerman
# Copyright © 2006 Frank Lichtenheld
# Copyright © 2006-2010,2012-2013 Guillem Jover <guillem@debian.org>
# Copyright © 2007 Raphaël Hertzog
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

use strict;
use warnings;

use POSIX qw(:errno_h);
use Cwd qw(realpath);
use File::Basename qw(dirname);

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Util qw(:list);
use Dpkg::Path qw(relative_to_pkg_root guess_pkg_root_dir
		  check_files_are_the_same get_control_path);
use Dpkg::Version;
use Dpkg::Shlibs qw(find_library get_library_paths);
use Dpkg::Shlibs::Objdump;
use Dpkg::Shlibs::SymbolFile;
use Dpkg::Arch qw(get_host_arch);
use Dpkg::Deps;
use Dpkg::Control::Info;
use Dpkg::Control::Fields;


use constant {
    WARN_SYM_NOT_FOUND => 1,
    WARN_DEP_AVOIDABLE => 2,
    WARN_NOT_NEEDED => 4,
};

# By increasing importance
my @depfields = qw(Suggests Recommends Depends Pre-Depends);
my $i = 0; my %depstrength = map { $_ => $i++ } @depfields;

textdomain('dpkg-dev');

my $admindir = $Dpkg::ADMINDIR;
my $shlibsoverride = "$Dpkg::CONFDIR/shlibs.override";
my $shlibsdefault = "$Dpkg::CONFDIR/shlibs.default";
my $shlibslocal = 'debian/shlibs.local';
my $packagetype = 'deb';
my $dependencyfield = 'Depends';
my $varlistfile = 'debian/substvars';
my $varnameprefix = 'shlibs';
my $ignore_missing_info = 0;
my $warnings = 3;
my $debug = 0;
my @exclude = ();
my @pkg_dir_to_search = ();
my $host_arch = get_host_arch();

my (@pkg_shlibs, @pkg_symbols, @pkg_root_dirs);
if (-d 'debian') {
    push @pkg_symbols, glob 'debian/*/DEBIAN/symbols';
    push @pkg_shlibs, glob 'debian/*/DEBIAN/shlibs';
    my %uniq = map { guess_pkg_root_dir($_) => 1 } (@pkg_symbols, @pkg_shlibs);
    push @pkg_root_dirs, keys %uniq;
}

my ($stdout, %exec);
foreach (@ARGV) {
    if (m/^-T(.*)$/) {
	$varlistfile = $1;
    } elsif (m/^-p(\w[-:0-9A-Za-z]*)$/) {
	$varnameprefix = $1;
    } elsif (m/^-L(.*)$/) {
	$shlibslocal = $1;
    } elsif (m/^-l(.*)$/) {
	Dpkg::Shlibs::add_library_dir($1);
    } elsif (m/^-S(.*)$/) {
	push @pkg_dir_to_search, $1;
    } elsif (m/^-O$/) {
	$stdout = 1;
    } elsif (m/^-O(.+)$/) {
	$varlistfile = $1;
    } elsif (m/^-(\?|-help)$/) {
	usage(); exit(0);
    } elsif (m/^--version$/) {
	version(); exit(0);
    } elsif (m/^--admindir=(.*)$/) {
	$admindir = $1;
	if (not -d $admindir) {
	    error(_g("administrative directory '%s' does not exist"), $admindir);
	}
	$ENV{DPKG_ADMINDIR} = $admindir;
    } elsif (m/^-d(.*)$/) {
	$dependencyfield = field_capitalize($1);
	if (not defined $depstrength{$dependencyfield}) {
	    warning(_g("unrecognized dependency field '%s'"), $dependencyfield);
	}
    } elsif (m/^-e(.*)$/) {
	if (exists $exec{$1}) {
	    # Affect the binary to the most important field
	    if ($depstrength{$dependencyfield} > $depstrength{$exec{$1}}) {
		$exec{$1} = $dependencyfield;
	    }
	} else {
	    $exec{$1} = $dependencyfield;
	}
    } elsif (m/^--ignore-missing-info$/) {
	$ignore_missing_info = 1;
    } elsif (m/^--warnings=(\d+)$/) {
	$warnings = $1;
    } elsif (m/^-t(.*)$/) {
	$packagetype = $1;
    } elsif (m/^-v$/) {
	$debug++;
    } elsif (m/^-x(.*)$/) {
	push @exclude, $1;
    } elsif (m/^-/) {
	usageerr(_g("unknown option \`%s'"), $_);
    } else {
	if (exists $exec{$_}) {
	    # Affect the binary to the most important field
	    if ($depstrength{$dependencyfield} > $depstrength{$exec{$_}}) {
		$exec{$_} = $dependencyfield;
	    }
	} else {
	    $exec{$_} = $dependencyfield;
	}
    }
}
usageerr(_g('need at least one executable')) unless scalar keys %exec;

my $control = Dpkg::Control::Info->new();
my $fields = $control->get_source();
my $bd_value = deps_concat($fields->{'Build-Depends'}, $fields->{'Build-Depends-Arch'});
my $build_deps = deps_parse($bd_value, build_dep => 1, reduce_restrictions => 1);
error(_g('error occurred while parsing %s'), 'Build-Depends/Build-Depends-Arch')
    unless defined $build_deps;

my %dependencies;

# Statictics on soname seen in the whole run (with multiple analysis of
# binaries)
my %global_soname_notfound;
my %global_soname_used;
my %global_soname_needed;

# Symfile and objdump caches
my %symfile_cache;
my %objdump_cache;
my %symfile_has_soname_cache;

# Used to count errors due to missing libraries
my $error_count = 0;

my $cur_field;
foreach my $file (keys %exec) {
    $cur_field = $exec{$file};
    print ">> Scanning $file (for $cur_field field)\n" if $debug;

    my $obj = Dpkg::Shlibs::Objdump::Object->new($file);
    my @sonames = $obj->get_needed_libraries;

    # Load symbols files for all needed libraries (identified by SONAME)
    my %libfiles;
    my %altlibfiles;
    my %soname_notfound;
    my %alt_soname;
    foreach my $soname (@sonames) {
	my $lib = my_find_library($soname, $obj->{RPATH}, $obj->{format}, $file);
	unless (defined $lib) {
	    $soname_notfound{$soname} = 1;
	    $global_soname_notfound{$soname} = 1;
	    my $msg = _g("couldn't find library %s needed by %s (ELF " .
			 "format: '%s'; RPATH: '%s')");
	    if (scalar(split_soname($soname))) {
		errormsg($msg, $soname, $file, $obj->{format}, join(':', @{$obj->{RPATH}}));
		$error_count++;
	    } else {
		warning($msg, $soname, $file, $obj->{format}, join(':', @{$obj->{RPATH}}));
	    }
	    next;
	}
	$libfiles{$lib} = $soname;
	my $reallib = realpath($lib);
	if ($reallib ne $lib) {
	    $altlibfiles{$reallib} = $soname;
	}
	print "Library $soname found in $lib\n" if $debug;
    }
    my $file2pkg = find_packages(keys %libfiles, keys %altlibfiles);
    my $symfile = Dpkg::Shlibs::SymbolFile->new();
    my $dumplibs_wo_symfile = Dpkg::Shlibs::Objdump->new();
    my @soname_wo_symfile;
    foreach my $lib (keys %libfiles) {
	my $soname = $libfiles{$lib};

	if (none { $_ ne '' } @{$file2pkg->{$lib}}) {
	    # The path of the library as calculated is not the
	    # official path of a packaged file, try to fallback on
	    # on the realpath() first, maybe this one is part of a package
	    my $reallib = realpath($lib);
	    if (exists $file2pkg->{$reallib}) {
		$file2pkg->{$lib} = $file2pkg->{$reallib};
	    }
	}
	if (none { $_ ne '' } @{$file2pkg->{$lib}}) {
	    # If the library is really not available in an installed package,
	    # it's because it's in the process of being built
	    # Empty package name will lead to consideration of symbols
	    # file from the package being built only
	    $file2pkg->{$lib} = [''];
	    print "No associated package found for $lib\n" if $debug;
	}

	# Load symbols/shlibs files from packages providing libraries
	foreach my $pkg (@{$file2pkg->{$lib}}) {
	    my $symfile_path;
            my $haslocaldep = 0;
            if (-e $shlibslocal and
                defined(extract_from_shlibs($soname, $shlibslocal)))
            {
                $haslocaldep = 1;
            }
            if ($packagetype eq 'deb' and not $haslocaldep) {
		# Use fine-grained dependencies only on real deb
                # and only if the dependency is not provided by shlibs.local
		$symfile_path = find_symbols_file($pkg, $soname, $lib);
            }
            if (defined($symfile_path)) {
                # Load symbol information
                print "Using symbols file $symfile_path for $soname\n" if $debug;
                unless (exists $symfile_cache{$symfile_path}) {
                    $symfile_cache{$symfile_path} =
                        Dpkg::Shlibs::SymbolFile->new(file => $symfile_path);
                }
                $symfile->merge_object_from_symfile($symfile_cache{$symfile_path}, $soname);
            }
	    if (defined($symfile_path) && $symfile->has_object($soname)) {
		# Initialize dependencies with the smallest minimal version
                # of all symbols (unversioned dependency is not ok as the
                # library might not have always been available in the
                # package and we really need it)
		my $dep = $symfile->get_dependency($soname);
		my $minver = $symfile->get_smallest_version($soname) || '';
		foreach my $subdep (split /\s*,\s*/, $dep) {
		    if (not exists $dependencies{$cur_field}{$subdep}) {
			$dependencies{$cur_field}{$subdep} = Dpkg::Version->new($minver);
                        print " Initialize dependency ($subdep) with minimal " .
                              "version ($minver)\n" if $debug > 1;
		    }
		}
	    } else {
		# No symbol file found, fall back to standard shlibs
                print "Using shlibs+objdump for $soname (file $lib)\n" if $debug;
                unless (exists $objdump_cache{$lib}) {
                    $objdump_cache{$lib} = Dpkg::Shlibs::Objdump::Object->new($lib);
                }
                my $libobj = $objdump_cache{$lib};
                my $id = $dumplibs_wo_symfile->add_object($libobj);
		if (($id ne $soname) and ($id ne $lib)) {
		    warning(_g('%s has an unexpected SONAME (%s)'), $lib, $id);
		    $alt_soname{$id} = $soname;
		}
		push @soname_wo_symfile, $soname;
		# Only try to generate a dependency for libraries with a SONAME
		if ($libobj->is_public_library() and not
		    add_shlibs_dep($soname, $pkg, $lib)) {
		    # This failure is fairly new, try to be kind by
		    # ignoring as many cases that can be safely ignored
		    my $ignore = 0;
		    # 1/ when the lib and the binary are in the same
		    # package
		    my $root_file = guess_pkg_root_dir($file);
		    my $root_lib = guess_pkg_root_dir($lib);
		    $ignore++ if defined $root_file and defined $root_lib
			and check_files_are_the_same($root_file, $root_lib);
		    # 2/ when the lib is not versioned and can't be
		    # handled by shlibs
		    $ignore++ unless scalar(split_soname($soname));
		    # 3/ when we have been asked to do so
		    $ignore++ if $ignore_missing_info;
		    error(_g('no dependency information found for %s ' .
		             '(used by %s)'), $lib, $file)
		        unless $ignore;
		}
	    }
	}
    }

    # Scan all undefined symbols of the binary and resolve to a
    # dependency
    my %soname_used;
    foreach (@sonames) {
        # Initialize statistics
        $soname_used{$_} = 0;
        $global_soname_used{$_} = 0 unless exists $global_soname_used{$_};
        if (exists $global_soname_needed{$_}) {
            push @{$global_soname_needed{$_}}, $file;
        } else {
            $global_soname_needed{$_} = [ $file ];
        }
    }
    my $nb_warnings = 0;
    my $nb_skipped_warnings = 0;
    # Disable warnings about missing symbols when we have not been able to
    # find all libs
    my $disable_warnings = scalar(keys(%soname_notfound));
    my $in_public_dir = 1;
    if (my $relname = relative_to_pkg_root($file)) {
        my $parent_dir = '/' . dirname($relname);
        $in_public_dir = any { $parent_dir eq $_ } get_library_paths();
    } else {
        warning(_g('binaries to analyze should already be ' .
                   "installed in their package's directory"));
    }
    print "Analyzing all undefined symbols\n" if $debug > 1;
    foreach my $sym ($obj->get_undefined_dynamic_symbols()) {
	my $name = $sym->{name};
	if ($sym->{version}) {
	    $name .= '@' . "$sym->{version}";
	} else {
	    $name .= '@' . 'Base';
	}
        print " Looking up symbol $name\n" if $debug > 1;
	my %symdep = $symfile->lookup_symbol($name, \@sonames);
	if (keys %symdep) {
	    my $depends = $symfile->get_dependency($symdep{soname},
		$symdep{symbol}{dep_id});
            print " Found in symbols file of $symdep{soname} (minver: " .
                  "$symdep{symbol}{minver}, dep: $depends)\n" if $debug > 1;
	    $soname_used{$symdep{soname}}++;
	    $global_soname_used{$symdep{soname}}++;
            if (exists $alt_soname{$symdep{soname}}) {
                # Also count usage on alternate soname
                $soname_used{$alt_soname{$symdep{soname}}}++;
                $global_soname_used{$alt_soname{$symdep{soname}}}++;
            }
	    update_dependency_version($depends, $symdep{symbol}{minver});
	} else {
	    my $syminfo = $dumplibs_wo_symfile->locate_symbol($name);
	    if (not defined($syminfo)) {
                print " Not found\n" if $debug > 1;
                next unless ($warnings & WARN_SYM_NOT_FOUND);
		next if $disable_warnings;
		# Complain about missing symbols only for executables
		# and public libraries
		if ($obj->is_executable() or $obj->is_public_library()) {
		    my $print_name = $name;
		    # Drop the default suffix for readability
		    $print_name =~ s/\@Base$//;
		    unless ($sym->{weak}) {
			if ($debug or ($in_public_dir and $nb_warnings < 10)
                            or (not $in_public_dir and $nb_warnings < 1))
                        {
                            if ($in_public_dir) {
			        warning(_g('symbol %s used by %s found in none of the ' .
				           'libraries'), $print_name, $file);
                            } else {
			        warning(_g('%s contains an unresolvable reference to ' .
                                           "symbol %s: it's probably a plugin"),
                                        $file, $print_name);
                            }
			    $nb_warnings++;
			} else {
			    $nb_skipped_warnings++;
			}
		    }
		}
	    } else {
                print " Found in $syminfo->{soname} ($syminfo->{objid})\n" if $debug > 1;
		if (exists $alt_soname{$syminfo->{soname}}) {
		    # Also count usage on alternate soname
		    $soname_used{$alt_soname{$syminfo->{soname}}}++;
		    $global_soname_used{$alt_soname{$syminfo->{soname}}}++;
		}
		$soname_used{$syminfo->{soname}}++;
		$global_soname_used{$syminfo->{soname}}++;
	    }
	}
    }
    warning(P_('%d similar warning has been skipped (use -v to see it)',
               '%d other similar warnings have been skipped (use -v to see ' .
               'them all)', $nb_skipped_warnings), $nb_skipped_warnings)
        if $nb_skipped_warnings;
    foreach my $soname (@sonames) {
	# Adjust minimal version of dependencies with information
	# extracted from build-dependencies
	my $dev_pkg = $symfile->get_field($soname, 'Build-Depends-Package');
	if (defined $dev_pkg) {
            print "Updating dependencies of $soname with build-dependencies\n" if $debug;
	    my $minver = get_min_version_from_deps($build_deps, $dev_pkg);
	    if (defined $minver) {
		foreach my $dep ($symfile->get_dependencies($soname)) {
		    update_dependency_version($dep, $minver, 1);
                    print " Minimal version of $dep updated with $minver\n" if $debug;
		}
	    } else {
                print " No minimal version found in $dev_pkg build-dependency\n" if $debug;
            }
	}

	# Warn about un-NEEDED libraries
	unless ($soname_notfound{$soname} or $soname_used{$soname}) {
	    # Ignore warning for libm.so.6 if also linked against libstdc++
	    next if ($soname =~ /^libm\.so\.\d+$/ and
	             any { m/^libstdc\+\+\.so\.\d+/ } @sonames);
            next unless ($warnings & WARN_NOT_NEEDED);
	    warning(_g('%s should not be linked against %s (it uses none of ' .
	               "the library's symbols)"), $file, $soname);
	}
    }
}

# Warn of unneeded libraries at the "package" level (i.e. over all
# binaries that we have inspected)
foreach my $soname (keys %global_soname_needed) {
    unless ($global_soname_notfound{$soname} or $global_soname_used{$soname}) {
        next if ($soname =~ /^libm\.so\.\d+$/ and
                 any { m/^libstdc\+\+\.so\.\d+/ } keys %global_soname_needed);
        next unless ($warnings & WARN_DEP_AVOIDABLE);
        warning(P_('package could avoid a useless dependency if %s was not ' .
                   "linked against %s (it uses none of the library's symbols)",
                   'package could avoid a useless dependency if %s were not ' .
                   "linked against %s (they use none of the library's symbols)",
                   scalar @{$global_soname_needed{$soname}}),
                join(' ', @{$global_soname_needed{$soname}}), $soname);
    }
}

# Quit now if any missing libraries
if ($error_count >= 1) {
    my $note = _g('Note: libraries are not searched in other binary packages ' .
	"that do not have any shlibs or symbols file.\nTo help dpkg-shlibdeps " .
	'find private libraries, you might need to use -l.');
    error(P_('cannot continue due to the error above',
             'cannot continue due to the errors listed above',
             $error_count) . "\n" . $note);
}

# Open substvars file
my $fh;
if ($stdout) {
    $fh = \*STDOUT;
} else {
    open(my $new_fh, '>', "$varlistfile.new")
        or syserr(_g("open new substvars file \`%s'"), "$varlistfile.new");
    if (-e $varlistfile) {
	open(my $old_fh, '<', $varlistfile)
	    or syserr(_g("open old varlist file \`%s' for reading"), $varlistfile);
	while (my $entry = <$old_fh>) {
	    next if $entry =~ m/^\Q$varnameprefix\E:/;
	    print { $new_fh } $entry
	        or syserr(_g("copy old entry to new varlist file \`%s'"),
	                  "$varlistfile.new");
	}
	close($old_fh);
    }
    $fh = $new_fh;
}

# Write out the shlibs substvars
my %depseen;

sub filter_deps {
    my ($dep, $field) = @_;
    # Skip dependencies on excluded packages
    foreach my $exc (@exclude) {
	return 0 if $dep =~ /^\s*\Q$exc\E\b/;
    }
    # Don't include dependencies if they are already
    # mentionned in a higher priority field
    if (not exists($depseen{$dep})) {
	$depseen{$dep} = $dependencies{$field}{$dep};
	return 1;
    } else {
	# Since dependencies can be versionned, we have to
	# verify if the dependency is stronger than the
	# previously seen one
	my $stronger;
	if ($depseen{$dep} eq $dependencies{$field}{$dep}) {
	    # If both versions are the same (possibly unversionned)
	    $stronger = 0;
	} elsif ($dependencies{$field}{$dep} eq '') {
	    $stronger = 0; # If the dep is unversionned
	} elsif ($depseen{$dep} eq '') {
	    $stronger = 1; # If the dep seen is unversionned
	} elsif (version_compare_relation($depseen{$dep}, REL_GT,
                                          $dependencies{$field}{$dep})) {
	    # The version of the dep seen is stronger...
	    $stronger = 0;
	} else {
	    $stronger = 1;
	}
	$depseen{$dep} = $dependencies{$field}{$dep} if $stronger;
	return $stronger;
    }
}

foreach my $field (reverse @depfields) {
    my $dep = '';
    if (exists $dependencies{$field} and scalar keys %{$dependencies{$field}}) {
	$dep = join ', ',
	    map {
		# Translate dependency templates into real dependencies
		if ($dependencies{$field}{$_}) {
		    s/#MINVER#/(>= $dependencies{$field}{$_})/g;
		} else {
		    s/#MINVER#//g;
		}
		s/\s+/ /g;
		$_;
	    } grep { filter_deps($_, $field) }
	    keys %{$dependencies{$field}};
    }
    if ($dep) {
        my $obj = deps_parse($dep);
        error(_g('invalid dependency got generated: %s'), $dep) unless defined $obj;
        $obj->sort();
	print { $fh } "$varnameprefix:$field=$obj\n";
    }
}

# Replace old file by new one
if (!$stdout) {
    close($fh) or syserr(_g('cannot close %s'), "$varlistfile.new");
    rename("$varlistfile.new",$varlistfile)
        or syserr(_g("install new varlist file \`%s'"), $varlistfile);
}

##
## Functions
##

sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    printf _g('
This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf _g(
'Usage: %s [<option>...] <executable>|-e<executable> [<option>...]')
    . "\n\n" . _g(
"Positional options (order is significant):
  <executable>             include dependencies for <executable>,
  -e<executable>           (use -e if <executable> starts with '-')
  -d<dependency-field>     next executable(s) set shlibs:<dependency-field>.")
    . "\n\n" . _g(
"Options:
  -l<library-dir>          add directory to private shared library search list.
  -p<varname-prefix>       set <varname-prefix>:* instead of shlibs:*.
  -O[<file>]               write variable settings to stdout (or <file>).
  -L<local-shlibs-file>    shlibs override file, not debian/shlibs.local.
  -T<substvars-file>       update variables here, not debian/substvars.
  -t<type>                 set package type (default is deb).
  -x<package>              exclude package from the generated dependencies.
  -S<package-build-dir>    search needed libraries in the given
                           package build directory first.
  -v                       enable verbose mode (can be used multiple times).
  --ignore-missing-info    don't fail if dependency information can't be found.
  --warnings=<value>       define set of active warnings (see manual page).
  --admindir=<directory>   change the administrative directory.
  -?, --help               show this help message.
      --version            show the version.")
    . "\n\n" . _g(
'Dependency fields recognized are:
  %s
'), $Dpkg::PROGNAME, join('/', @depfields);
}

sub get_min_version_from_deps {
    my ($dep, $pkg) = @_;
    if ($dep->isa('Dpkg::Deps::Simple')) {
	if (($dep->{package} eq $pkg) &&
	    defined($dep->{relation}) &&
	    (($dep->{relation} eq REL_GE) ||
	     ($dep->{relation} eq REL_GT)))
	{
	    return $dep->{version};
	}
	return;
    } else {
	my $res;
	foreach my $subdep ($dep->get_deps()) {
	    my $minver = get_min_version_from_deps($subdep, $pkg);
	    next if not defined $minver;
	    if (defined $res) {
		if (version_compare_relation($minver, REL_GT, $res)) {
		    $res = $minver;
		}
	    } else {
		$res = $minver;
	    }
	}
	return $res;
    }
}

sub update_dependency_version {
    my ($dep, $minver, $existing_only) = @_;
    return if not defined($minver);
    $minver = Dpkg::Version->new($minver);
    foreach my $subdep (split /\s*,\s*/, $dep) {
	if (exists $dependencies{$cur_field}{$subdep} and
	    defined($dependencies{$cur_field}{$subdep}))
	{
	    if ($dependencies{$cur_field}{$subdep} eq '' or
		version_compare_relation($minver, REL_GT,
				         $dependencies{$cur_field}{$subdep}))
	    {
		$dependencies{$cur_field}{$subdep} = $minver;
	    }
	} elsif (!$existing_only) {
	    $dependencies{$cur_field}{$subdep} = $minver;
	}
    }
}

sub add_shlibs_dep {
    my ($soname, $pkg, $libfile) = @_;
    my @shlibs = ($shlibslocal, $shlibsoverride);
    if ($pkg eq '') {
	# If the file is not packaged, try to find out the shlibs file in
	# the package being built where the lib has been found
	my $pkg_root = guess_pkg_root_dir($libfile);
	if (defined $pkg_root) {
	    push @shlibs, "$pkg_root/DEBIAN/shlibs";
	}
	# Fallback to other shlibs files but it shouldn't be necessary
	push @shlibs, @pkg_shlibs;
    } else {
	my $control_file = get_control_path($pkg, 'shlibs');
	push @shlibs, $control_file if defined $control_file;
    }
    push @shlibs, $shlibsdefault;
    print " Looking up shlibs dependency of $soname provided by '$pkg'\n" if $debug;
    foreach my $file (@shlibs) {
	next if not -e $file;
	my $dep = extract_from_shlibs($soname, $file);
	if (defined($dep)) {
	    print " Found $dep in $file\n" if $debug;
	    foreach (split(/,\s*/, $dep)) {
		# Note: the value is empty for shlibs based dependency
		# symbol based dependency will put a valid version as value
		$dependencies{$cur_field}{$_} = Dpkg::Version->new('');
	    }
	    return 1;
	}
    }
    print " Found nothing\n" if $debug;
    return 0;
}

sub split_soname {
    my $soname = shift;
    if ($soname =~ /^(.*)\.so\.(.*)$/) {
	return wantarray ? ($1, $2) : 1;
    } elsif ($soname =~ /^(.*)-(\d.*)\.so$/) {
	return wantarray ? ($1, $2) : 1;
    } else {
	return wantarray ? () : 0;
    }
}

sub extract_from_shlibs {
    my ($soname, $shlibfile) = @_;
    # Split soname in name/version
    my ($libname, $libversion) = split_soname($soname);
    unless (defined $libname) {
	warning(_g("can't extract name and version from library name '%s'"),
	        $soname);
	return;
    }
    # Open shlibs file
    open(my $shlibs_fh, '<', $shlibfile)
        or syserr(_g("unable to open shared libs info file \`%s'"), $shlibfile);
    my $dep;
    while (<$shlibs_fh>) {
	s/\s*\n$//;
	next if m/^\#/;
	if (!m/^\s*(?:(\S+):\s+)?(\S+)\s+(\S+)(?:\s+(\S.*\S))?\s*$/) {
	    warning(_g("shared libs info file \`%s' line %d: bad line \`%s'"),
	            $shlibfile, $., $_);
	    next;
	}
	my $depread = defined($4) ? $4 : '';
	if (($libname eq $2) && ($libversion eq $3)) {
	    # Define dep and end here if the package type explicitly
	    # matches. Otherwise if the packagetype is not specified, use
	    # the dep only as a default that can be overriden by a later
	    # line
	    if (defined($1)) {
		if ($1 eq $packagetype) {
		    $dep = $depread;
		    last;
		}
	    } else {
		$dep //= $depread;
	    }
	}
    }
    close($shlibs_fh);
    return $dep;
}

sub find_symbols_file {
    my ($pkg, $soname, $libfile) = @_;
    my @files;
    if ($pkg eq '') {
	# If the file is not packaged, try to find out the symbols file in
	# the package being built where the lib has been found
	my $pkg_root = guess_pkg_root_dir($libfile);
	if (defined $pkg_root) {
	    push @files, "$pkg_root/DEBIAN/symbols";
	}
	# Fallback to other symbols files but it shouldn't be necessary
	push @files, @pkg_symbols;
    } else {
	push @files, "$Dpkg::CONFDIR/symbols/$pkg.symbols.$host_arch",
	    "$Dpkg::CONFDIR/symbols/$pkg.symbols";
	my $control_file = get_control_path($pkg, 'symbols');
	push @files, $control_file if defined $control_file;
    }

    foreach my $file (@files) {
	if (-e $file and symfile_has_soname($file, $soname)) {
	    return $file;
	}
    }
    return;
}

sub symfile_has_soname {
    my ($file, $soname) = @_;

    if (exists $symfile_has_soname_cache{$file}{$soname}) {
        return $symfile_has_soname_cache{$file}{$soname};
    }

    open(my $symfile_fh, '<', $file)
        or syserr(_g('cannot open file %s'), $file);
    my $result = 0;
    while (<$symfile_fh>) {
	if (/^\Q$soname\E /) {
	    $result = 1;
	    last;
	}
    }
    close($symfile_fh);
    $symfile_has_soname_cache{$file}{$soname} = $result;
    return $result;
}

# find_library ($soname, \@rpath, $format)
sub my_find_library {
    my ($lib, $rpath, $format, $execfile) = @_;
    my $file;

    # Create real RPATH in case $ORIGIN is used
    # Note: ld.so also supports $PLATFORM and $LIB but they are
    # used in real case (yet)
    my $libdir = relative_to_pkg_root($execfile);
    my $origin;
    if (defined $libdir) {
	$origin = "/$libdir";
	$origin =~ s{/+[^/]*$}{};
    }
    my @RPATH = ();
    foreach my $path (@{$rpath}) {
	if ($path =~ /\$ORIGIN|\$\{ORIGIN\}/) {
	    if (defined $origin) {
		$path =~ s/\$ORIGIN/$origin/g;
		$path =~ s/\$\{ORIGIN\}/$origin/g;
	    } else {
		warning(_g('$ORIGIN is used in RPATH of %s and the corresponding ' .
		'directory could not be identified due to lack of DEBIAN ' .
		"sub-directory in the root of package's build tree"), $execfile);
	    }
	}
	push @RPATH, $path;
    }

    # Look into the packages we're currently building in the following
    # order:
    # - package build tree of the binary which is analyzed
    # - package build tree given on the command line (option -S)
    # - other package build trees that contain either a shlibs or a
    # symbols file
    my @builddirs;
    my $pkg_root = guess_pkg_root_dir($execfile);
    push @builddirs, $pkg_root if defined $pkg_root;
    push @builddirs, @pkg_dir_to_search;
    push @builddirs, @pkg_root_dirs;
    my %dir_checked;
    foreach my $builddir (@builddirs) {
	next if defined($dir_checked{$builddir});
	$file = find_library($lib, \@RPATH, $format, $builddir);
	return $file if defined($file);
	$dir_checked{$builddir} = 1;
    }

    # Fallback in the root directory if we have not found what we were
    # looking for in the packages
    $file = find_library($lib, \@RPATH, $format, '');
    return $file if defined($file);

    return;
}

my %cached_pkgmatch = ();

sub find_packages {
    my @files;
    my $pkgmatch = {};

    foreach (@_) {
	if (exists $cached_pkgmatch{$_}) {
	    $pkgmatch->{$_} = $cached_pkgmatch{$_};
	} else {
	    push @files, $_;
	    $cached_pkgmatch{$_} = ['']; # placeholder to cache misses too.
	    $pkgmatch->{$_} = [''];        # might be replaced later on
	}
    }
    return $pkgmatch unless scalar(@files);

    my $pid = open(my $dpkg_fh, '-|');
    syserr(_g('cannot fork for %s'), 'dpkg --search') unless defined($pid);
    if (!$pid) {
	# Child process running dpkg --search and discarding errors
	close STDERR;
	open STDERR, '>', '/dev/null'
	    or syserr(_g('cannot open file %s'), '/dev/null');
	$ENV{LC_ALL} = 'C';
	exec('dpkg', '--search', '--', @files)
	    or syserr(_g('unable to execute %s'), 'dpkg');
    }
    while (defined($_ = <$dpkg_fh>)) {
	chomp($_);
	if (m/^local diversion |^diversion by/) {
	    warning(_g('diversions involved - output may be incorrect'));
	    print { *STDERR } " $_\n"
		or syserr(_g('write diversion info to stderr'));
	} elsif (m/^([-a-z0-9+.:, ]+): (\/.*)$/) {
	    $cached_pkgmatch{$2} = $pkgmatch->{$2} = [ split(/, /, $1) ];
	} else {
	    warning(_g("unknown output from dpkg --search: '%s'"), $_);
	}
    }
    close($dpkg_fh);
    return $pkgmatch;
}
