#!/usr/bin/perl
#
# dpkg-gencontrol
#
# Copyright © 1996 Ian Jackson
# Copyright © 2000,2002 Wichert Akkerman
# Copyright © 2006-2013 Guillem Jover <guillem@debian.org>
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

use POSIX qw(:errno_h :fcntl_h);
use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Util qw(:list);
use Dpkg::File;
use Dpkg::Arch qw(get_host_arch debarch_eq debarch_is);
use Dpkg::Package;
use Dpkg::BuildProfiles qw(get_build_profiles);
use Dpkg::Deps;
use Dpkg::Control;
use Dpkg::Control::Info;
use Dpkg::Control::Fields;
use Dpkg::Substvars;
use Dpkg::Vars;
use Dpkg::Changelog::Parse;

textdomain('dpkg-dev');


my $controlfile = 'debian/control';
my $changelogfile = 'debian/changelog';
my $changelogformat;
my $fileslistfile = 'debian/files';
my $packagebuilddir = 'debian/tmp';
my $outputfile;

my $sourceversion;
my $binaryversion;
my $forceversion;
my $forcefilename;
my $stdout;
my %remove;
my %override;
my $oppackage;
my $substvars = Dpkg::Substvars->new();
my $substvars_loaded = 0;


sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    printf _g('
This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf _g(
'Usage: %s [<option>...]')
    . "\n\n" . _g(
'Options:
  -p<package>              print control file for package.
  -c<control-file>         get control info from this file.
  -l<changelog-file>       get per-version info from this file.
  -F<changelog-format>     force changelog format.
  -v<force-version>        set version of binary package.
  -f<files-list-file>      write files here instead of debian/files.
  -P<package-build-dir>    temporary build dir instead of debian/tmp.
  -n<filename>             assume the package filename will be <filename>.
  -O[<file>]               write to stdout (or <file>), not .../DEBIAN/control.
  -is, -ip, -isp, -ips     deprecated, ignored for compatibility.
  -D<field>=<value>        override or add a field and value.
  -U<field>                remove a field.
  -V<name>=<value>         set a substitution variable.
  -T<substvars-file>       read variables here, not debian/substvars.
  -?, --help               show this help message.
      --version            show the version.
'), $Dpkg::PROGNAME;
}

while (@ARGV) {
    $_=shift(@ARGV);
    if (m/^-p/) {
        $oppackage = $';
        my $err = pkg_name_is_illegal($oppackage);
        error(_g("illegal package name '%s': %s"), $oppackage, $err) if $err;
    } elsif (m/^-c/) {
        $controlfile= $';
    } elsif (m/^-l/) {
        $changelogfile= $';
    } elsif (m/^-P/) {
        $packagebuilddir= $';
    } elsif (m/^-f/) {
        $fileslistfile= $';
    } elsif (m/^-v(.+)$/) {
        $forceversion= $1;
    } elsif (m/^-O$/) {
        $stdout= 1;
    } elsif (m/^-O(.+)$/) {
        $outputfile = $1;
    } elsif (m/^-i[sp][sp]?$/) {
	# ignored for backwards compatibility
    } elsif (m/^-F([0-9a-z]+)$/) {
        $changelogformat=$1;
    } elsif (m/^-D([^\=:]+)[=:]/) {
        $override{$1}= $';
    } elsif (m/^-U([^\=:]+)$/) {
        $remove{$1}= 1;
    } elsif (m/^-V(\w[-:0-9A-Za-z]*)[=:]/) {
        $substvars->set_as_used($1, $');
    } elsif (m/^-T(.*)$/) {
	$substvars->load($1) if -e $1;
	$substvars_loaded = 1;
    } elsif (m/^-n/) {
        $forcefilename= $';
    } elsif (m/^-(\?|-help)$/) {
        usage();
        exit(0);
    } elsif (m/^--version$/) {
        version();
        exit(0);
    } else {
        usageerr(_g("unknown option \`%s'"), $_);
    }
}

umask 0022; # ensure sane default permissions for created files
my %options = (file => $changelogfile);
$options{changelogformat} = $changelogformat if $changelogformat;
my $changelog = changelog_parse(%options);
if ($changelog->{'Binary-Only'}) {
    $options{count} = 1;
    $options{offset} = 1;
    my $prev_changelog = changelog_parse(%options);
    $sourceversion = $prev_changelog->{'Version'};
} else {
    $sourceversion = $changelog->{'Version'};
}

if (defined $forceversion) {
    $binaryversion = $forceversion;
} else {
    $binaryversion = $changelog->{'Version'};
}

$substvars->set_version_substvars($sourceversion, $binaryversion);
$substvars->set_arch_substvars();
$substvars->load('debian/substvars') if -e 'debian/substvars' and not $substvars_loaded;
my $control = Dpkg::Control::Info->new($controlfile);
my $fields = Dpkg::Control->new(type => CTRL_PKG_DEB);

# Old-style bin-nmus change the source version submitted to
# set_version_substvars()
$sourceversion = $substvars->get('source:Version');

my $pkg;

if (defined($oppackage)) {
    $pkg = $control->get_pkg_by_name($oppackage);
    if (not defined $pkg) {
        error(_g('package %s not in control info'), $oppackage)
    }
} else {
    my @packages = map { $_->{'Package'} } $control->get_packages();
    if (@packages == 0) {
        error(_g('no package stanza found in control info'));
    } elsif (@packages > 1) {
        error(_g('must specify package since control info has many (%s)'),
              "@packages");
    }
    $pkg = $control->get_pkg_by_idx(1);
}
$substvars->set_msg_prefix(sprintf(_g('package %s: '), $pkg->{Package}));

# Scan source package
my $src_fields = $control->get_source();
foreach (keys %{$src_fields}) {
    if (m/^Source$/) {
	set_source_package($src_fields->{$_});
    } else {
        field_transfer_single($src_fields, $fields);
    }
}

# Scan binary package
foreach (keys %{$pkg}) {
    my $v = $pkg->{$_};
    if (field_get_dep_type($_)) {
	# Delay the parsing until later
    } elsif (m/^Architecture$/) {
	my $host_arch = get_host_arch();

	if (debarch_eq('all', $v)) {
	    $fields->{$_} = $v;
	} else {
	    my @archlist = split(/\s+/, $v);
	    my @invalid_archs = grep { m/[^\w-]/ } @archlist;
	    warning(ngettext("`%s' is not a legal architecture string.",
			     "`%s' are not legal architecture strings.",
			     scalar(@invalid_archs)),
		    join("' `", @invalid_archs))
		if @invalid_archs >= 1;
	    if (none { debarch_is($host_arch, $_) } @archlist) {
		error(_g("current host architecture '%s' does not " .
			 "appear in package's architecture list (%s)"),
		      $host_arch, "@archlist");
	    }
	    $fields->{$_} = $host_arch;
	}
    } else {
        field_transfer_single($pkg, $fields);
    }
}

# Scan fields of dpkg-parsechangelog
foreach (keys %{$changelog}) {
    my $v = $changelog->{$_};

    if (m/^Source$/) {
	set_source_package($v);
    } elsif (m/^Version$/) {
        # Already handled previously.
    } elsif (m/^Maintainer$/) {
        # That field must not be copied from changelog even if it's
        # allowed in the binary package control information
    } else {
        field_transfer_single($changelog, $fields);
    }
}

$fields->{'Version'} = $binaryversion;

# Process dependency fields in a second pass, now that substvars have been
# initialized.

my $facts = Dpkg::Deps::KnownFacts->new();
$facts->add_installed_package($fields->{'Package'}, $fields->{'Version'},
                              $fields->{'Architecture'}, $fields->{'Multi-Arch'});
if (exists $pkg->{'Provides'}) {
    my $provides = deps_parse($substvars->substvars($pkg->{'Provides'}, no_warn => 1),
                              reduce_restrictions => 1, union => 1);
    if (defined $provides) {
	foreach my $subdep ($provides->get_deps()) {
	    if ($subdep->isa('Dpkg::Deps::Simple')) {
		$facts->add_provided_package($subdep->{package},
                        $subdep->{relation}, $subdep->{version},
                        $fields->{'Package'});
	    }
	}
    }
}

my (@seen_deps);
foreach my $field (field_list_pkg_dep()) {
    # Arch: all can't be simplified as the host architecture is not known
    my $reduce_arch = debarch_eq('all', $pkg->{Architecture} || 'all') ? 0 : 1;
    if (exists $pkg->{$field}) {
	my $dep;
	my $field_value = $substvars->substvars($pkg->{$field},
	    msg_prefix => sprintf(_g('%s field of package %s: '), $field, $pkg->{Package}));
	if (field_get_dep_type($field) eq 'normal') {
	    $dep = deps_parse($field_value, use_arch => 1,
	                      reduce_arch => $reduce_arch,
	                      reduce_profiles => 1);
	    error(_g('error occurred while parsing %s field: %s'), $field,
                  $field_value) unless defined $dep;
	    $dep->simplify_deps($facts, @seen_deps);
	    # Remember normal deps to simplify even further weaker deps
	    push @seen_deps, $dep;
	} else {
	    $dep = deps_parse($field_value, use_arch => 1,
	                      reduce_arch => $reduce_arch,
	                      reduce_profiles => 1, union => 1);
	    error(_g('error occurred while parsing %s field: %s'), $field,
                  $field_value) unless defined $dep;
	    $dep->simplify_deps($facts);
            $dep->sort();
	}
	error(_g('the %s field contains an arch-specific dependency but the ' .
	         'package is architecture all'), $field)
	    if $dep->has_arch_restriction();
	$fields->{$field} = $dep->output();
	delete $fields->{$field} unless $fields->{$field}; # Delete empty field
    }
}

$fields->{'Built-For-Profiles'} = join ' ', get_build_profiles();

for my $f (qw(Package Version)) {
    error(_g('missing information for output field %s'), $f)
        unless defined $fields->{$f};
}
for my $f (qw(Maintainer Description Architecture)) {
    warning(_g('missing information for output field %s'), $f)
        unless defined $fields->{$f};
}
$oppackage = $fields->{'Package'};

my $pkg_type = $pkg->{'Package-Type'} ||
               $pkg->get_custom_field('Package-Type') || 'deb';

if ($pkg_type eq 'udeb') {
    delete $fields->{'Package-Type'};
    delete $fields->{'Homepage'};
} else {
    for my $f (qw(Subarchitecture Kernel-Version Installer-Menu-Item)) {
        warning(_g('%s package with udeb specific field %s'), $pkg_type, $f)
            if defined($fields->{$f});
    }
}

my $sourcepackage = get_source_package();
my $verdiff = $binaryversion ne $sourceversion;
if ($oppackage ne $sourcepackage || $verdiff) {
    $fields->{'Source'} = $sourcepackage;
    $fields->{'Source'} .= ' (' . $sourceversion . ')' if $verdiff;
}

if (!defined($substvars->get('Installed-Size'))) {
    my $c = open(my $du_fh, '-|');
    if (not defined $c) {
        syserr(_g('cannot fork for %s'), 'du');
    }
    if (!$c) {
        chdir("$packagebuilddir")
            or syserr(_g("chdir for du to \`%s'"), $packagebuilddir);
        exec('du', '-k', '-s', '--apparent-size', '.')
            or syserr(_g('unable to execute %s'), 'du');
    }
    my $duo = '';
    while (<$du_fh>) {
	$duo .= $_;
    }
    close($du_fh);
    subprocerr(_g("du in \`%s'"), $packagebuilddir) if $?;
    if ($duo !~ m/^(\d+)\s+\.$/) {
        error(_g("du gave unexpected output \`%s'"), $duo);
    }
    $substvars->set_as_used('Installed-Size', $1);
}
if (defined($substvars->get('Extra-Size'))) {
    my $size = $substvars->get('Extra-Size') + $substvars->get('Installed-Size');
    $substvars->set_as_used('Installed-Size', $size);
}
if (defined($substvars->get('Installed-Size'))) {
    $fields->{'Installed-Size'} = $substvars->get('Installed-Size');
}

for my $f (keys %override) {
    $fields->{$f} = $override{$f};
}
for my $f (keys %remove) {
    delete $fields->{$f};
}

# Obtain a lock on debian/control to avoid simultaneous updates
# of debian/files when parallel building is in use
my $lockfh;
my $lockfile = 'debian/control';
$lockfile = $controlfile if not -e $lockfile;

sysopen($lockfh, $lockfile, O_WRONLY)
    or syserr(_g('cannot write %s'), $lockfile);
file_lock($lockfh, $lockfile);

open(my $fileslistnew_fh, '>', "$fileslistfile.new")
    or syserr(_g('open new files list file'));
binmode($fileslistnew_fh);
if (open(my $fileslist_fh, '<', $fileslistfile)) {
    binmode($fileslist_fh);
    while (<$fileslist_fh>) {
        chomp;
        next if m/^([-+0-9a-z.]+)_[^_]+_([\w-]+)\.(a-z+) /
                && ($1 eq $oppackage)
	        && ($3 eq $pkg_type)
	        && (debarch_eq($2, $fields->{'Architecture'} || '')
		    || debarch_eq($2, 'all'));
        print { $fileslistnew_fh } "$_\n"
            or syserr(_g('copy old entry to new files list file'));
    }
    close($fileslist_fh) or syserr(_g('close old files list file'));
} elsif ($! != ENOENT) {
    syserr(_g('read old files list file'));
}
my $sversion = $fields->{'Version'};
$sversion =~ s/^\d+://;
$forcefilename //= sprintf('%s_%s_%s.%s', $oppackage, $sversion,
                           $fields->{'Architecture'} || '', $pkg_type);

print { $fileslistnew_fh }
      $substvars->substvars(sprintf("%s %s %s\n",
                                    $forcefilename,
                                    $fields->{'Section'} || '-',
                                    $fields->{'Priority'} || '-'))
    or syserr(_g('write new entry to new files list file'));
close($fileslistnew_fh) or syserr(_g('close new files list file'));
rename("$fileslistfile.new", $fileslistfile)
    or syserr(_g('install new files list file'));

# Release the lock
close($lockfh) or syserr(_g('cannot close %s'), $lockfile);

my $cf;
my $fh_output;
if (!$stdout) {
    $cf = $outputfile // "$packagebuilddir/DEBIAN/control";
    open($fh_output, '>', "$cf.new")
        or syserr(_g("cannot open new output control file \`%s'"), "$cf.new");
} else {
    $fh_output = \*STDOUT;
}

$fields->apply_substvars($substvars);
$fields->output($fh_output);

if (!$stdout) {
    close($fh_output) or syserr(_g('cannot close %s'), "$cf.new");
    rename("$cf.new", "$cf")
        or syserr(_g("cannot install output control file \`%s'"), $cf);
}

$substvars->warn_about_unused();
