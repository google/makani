#!/usr/bin/perl
#
# dpkg-buildpackage
#
# Copyright © 1996 Ian Jackson
# Copyright © 2000 Wichert Akkerman
# Copyright © 2006-2010,2012-2013 Guillem Jover <guillem@debian.org>
# Copyright © 2007 Frank Lichtenheld
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

use Cwd;
use File::Basename;
use POSIX qw(:sys_wait_h);

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::BuildOptions;
use Dpkg::BuildProfiles qw(set_build_profiles);
use Dpkg::Compression;
use Dpkg::Version;
use Dpkg::Changelog::Parse;
use Dpkg::Path qw(find_command);
use Dpkg::IPC;

textdomain('dpkg-dev');

sub showversion {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    print _g('
This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf _g(
'Usage: %s [<option>...]')
    . "\n\n" . _g(
'Options:
  -F (default)   normal full build (binaries and sources).
  -b             binary-only, do not build source.
  -B             binary-only, no arch-indep files.
  -A             binary-only, only arch-indep files.
  -S             source only, no binary files.
  -nc            do not clean source tree (implies -b).
  -tc            clean source tree when finished.
  -D (default)   check build dependencies and conflicts.
  -d             do not check build dependencies and conflicts.
  -P<profiles>   assume given build profiles as active (comma-separated list).
  -R<rules>      rules file to execute (default is debian/rules).
  -T<target>     call debian/rules <target> with the proper environment.
      --as-root  ensure -T calls the target with root rights.
  -j[<number>]   specify jobs to run simultaneously (passed to <rules>).
  -r<gain-root-command>
                 command to gain root privileges (default is fakeroot).
  -p<sign-command>
                 command to sign .dsc and/or .changes files
                   (default is gpg2 or gpg).
  -k<keyid>      the key to use for signing.
  -ap            add pause before starting signature process.
  -us            unsigned source package.
  -uc            unsigned .changes file.
      --force-sign
                 force signing the resulting files.
      --admindir=<directory>
                 change the administrative directory.
  -?, --help     show this help message.
      --version  show the version.')
    . "\n\n" . _g(
'Options passed to dpkg-architecture:
  -a<arch>       Debian architecture we build for.
  -t<system>     set GNU system type.')
    . "\n\n" . _g(
'Options passed to dpkg-genchanges:
  -si (default)  source includes orig if new upstream.
  -sa            uploaded source always includes orig.
  -sd            uploaded source is diff and .dsc only.
  -v<version>    changes since version <version>.
  -m<maint>      maintainer for package is <maint>.
  -e<maint>      maintainer for release is <maint>.
  -C<descfile>   changes are described in <descfile>.
      --changes-option=<opt>
                 pass option <opt> to dpkg-genchanges.')
    . "\n\n" . _g(
'Options passed to dpkg-source:
  -sn            force Debian native source format.
  -s[sAkurKUR]   see dpkg-source for explanation.
  -z<level>      compression level to use for source.
  -Z<compressor> compression to use for source (gz|xz|bzip2|lzma).
  -i[<regex>]    ignore diffs of files matching regex.
  -I[<pattern>]  filter out files when building tarballs.
      --source-option=<opt>
                 pass option <opt> to dpkg-source.
'), $Dpkg::PROGNAME;
}

my @debian_rules = ('debian/rules');
my @rootcommand = ();
my $signcommand;
my ($admindir, $usepause, $noclean,
    $cleansource, $since, $maint,
    $changedby, $desc, $parallel);
my $checkbuilddep = 1;
my $signkey = defined $ENV{DEB_SIGN_KEYID} ? $ENV{DEB_SIGN_KEYID} : undef;
my $signforce = 0;
my $signreleased = 1;
my $signsource = 1;
my $signchanges = 1;
my $buildtarget = 'build';
my $binarytarget = 'binary';
my $targetarch = my $targetgnusystem = '';
my @build_profiles = ();
my $call_target = '';
my $call_target_as_root = 0;
my (@checkbuilddep_opts, @changes_opts, @source_opts);

use constant BUILD_DEFAULT    => 1;
use constant BUILD_SOURCE     => 2;
use constant BUILD_ARCH_DEP   => 4;
use constant BUILD_ARCH_INDEP => 8;
use constant BUILD_BINARY     => BUILD_ARCH_DEP | BUILD_ARCH_INDEP;
use constant BUILD_ALL        => BUILD_BINARY | BUILD_SOURCE;
my $include = BUILD_ALL | BUILD_DEFAULT;

sub build_normal() { return ($include & BUILD_ALL) == BUILD_ALL; }
sub build_sourceonly() { return $include == BUILD_SOURCE; }
sub build_binaryonly() { return !($include & BUILD_SOURCE); }
sub build_binaryindep() { return ($include == BUILD_ARCH_INDEP); }
sub build_opt() {
    return (($include == BUILD_BINARY) ? '-b' :
            (($include == BUILD_ARCH_DEP) ? '-B' :
             (($include == BUILD_ARCH_INDEP) ? '-A' :
              (($include == BUILD_SOURCE) ? '-S' :
               internerr("build_opt called with include=$include")))));
}

while (@ARGV) {
    $_ = shift @ARGV;

    if (/^(--help|-\?)$/) {
	usage;
	exit 0;
    } elsif (/^--version$/) {
	showversion;
	exit 0;
    } elsif (/^--admindir$/) {
        $admindir = shift @ARGV;
    } elsif (/^--admindir=(.*)$/) {
	$admindir = $1;
    } elsif (/^--source-option=(.*)$/) {
	push @source_opts, $1;
    } elsif (/^--changes-option=(.*)$/) {
	push @changes_opts, $1;
    } elsif (/^-j(\d*)$/) {
	$parallel = $1 || '';
    } elsif (/^-r(.*)$/) {
	@rootcommand = split /\s+/, $1;
    } elsif (/^-p(.*)$/) {
	$signcommand = $1;
    } elsif (/^-k(.*)$/) {
	$signkey = $1;
    } elsif (/^-([dD])$/) {
	$checkbuilddep = ($1 eq 'D');
    } elsif (/^-s(gpg|pgp)$/) {
	# Deprecated option
	warning(_g('-s%s is deprecated; always using gpg style interface'), $1);
    } elsif (/^--force-sign$/) {
	$signforce = 1;
    } elsif (/^-us$/) {
	$signsource = 0;
    } elsif (/^-uc$/) {
	$signchanges = 0;
    } elsif (/^-ap$/) {
	$usepause = 1;
    } elsif (/^-a(.*)$/) {
	$targetarch = $1;
    } elsif (/^-P(.*)$/) {
	@build_profiles = split /,/, $1;
    } elsif (/^-s[iad]$/) {
	push @changes_opts, $_;
    } elsif (/^-(?:s[insAkurKUR]|[zZ].*|i.*|I.*)$/) {
	push @source_opts, $_; # passed to dpkg-source
    } elsif (/^-tc$/) {
	$cleansource = 1;
    } elsif (/^-t(.*)$/) {
	$targetgnusystem = $1; # Order DOES matter!
    } elsif (/^(--target|-T)$/) {
        $call_target = shift @ARGV;
    } elsif (/^(--target=|-T)(.+)$/) {
        $call_target = $2;
    } elsif (/^--as-root$/) {
        $call_target_as_root = 1;
    } elsif (/^-nc$/) {
	$noclean = 1;
    } elsif (/^-b$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if build_sourceonly;
	$include = BUILD_BINARY;
	push @changes_opts, '-b';
	@checkbuilddep_opts = ();
	$buildtarget = 'build';
	$binarytarget = 'binary';
    } elsif (/^-B$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if build_sourceonly;
	$include = BUILD_ARCH_DEP;
	push @changes_opts, '-B';
	@checkbuilddep_opts = qw(-B);
	$buildtarget = 'build-arch';
	$binarytarget = 'binary-arch';
    } elsif (/^-A$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if build_sourceonly;
	$include = BUILD_ARCH_INDEP;
	push @changes_opts, '-A';
	@checkbuilddep_opts = qw(-A);
	$buildtarget = 'build-indep';
	$binarytarget = 'binary-indep';
    } elsif (/^-S$/) {
	usageerr(_g('cannot combine %s and %s'), build_opt, '-S')
	    if build_binaryonly;
	$include = BUILD_SOURCE;
	push @changes_opts, '-S';
	@checkbuilddep_opts = qw(-A -B);
    } elsif (/^-F$/) {
	usageerr(_g('cannot combine %s and %s'), $_, build_opt)
	    if not build_normal;
	$include = BUILD_ALL;
	@checkbuilddep_opts = ();
    } elsif (/^-v(.*)$/) {
	$since = $1;
    } elsif (/^-m(.*)$/) {
	$maint = $1;
    } elsif (/^-e(.*)$/) {
	$changedby = $1;
    } elsif (/^-C(.*)$/) {
	$desc = $1;
    } elsif (m/^-[EW]$/) {
	# Deprecated option
	warning(_g('-E and -W are deprecated, they are without effect'));
    } elsif (/^-R(.*)$/) {
	@debian_rules = split /\s+/, $1;
    } else {
	usageerr(_g('unknown option or argument %s'), $_);
    }
}

if ($noclean) {
    # -nc without -b/-B/-A/-S/-F implies -b
    $include = BUILD_BINARY if ($include & BUILD_DEFAULT);
}

if ($< == 0) {
    warning(_g('using a gain-root-command while being root')) if (@rootcommand);
} else {
    push @rootcommand, 'fakeroot' unless @rootcommand;

    if (!find_command($rootcommand[0])) {
	if ($rootcommand[0] eq 'fakeroot') {
	    error(_g("fakeroot not found, either install the fakeroot\n" .
	             'package, specify a command with the -r option, ' .
	             'or run this as root'));
	} else {
	    error(_g("gain-root-commmand '%s' not found"), $rootcommand[0]);
	}
    }
}

my $build_opts = Dpkg::BuildOptions->new();
if (defined $parallel) {
    $parallel = $build_opts->get('parallel') if $build_opts->has('parallel');
    $ENV{MAKEFLAGS} ||= '';
    $ENV{MAKEFLAGS} .= " -j$parallel";
    $build_opts->set('parallel', $parallel);
    $build_opts->export();
}

set_build_profiles(@build_profiles) if @build_profiles;

my $cwd = cwd();
my $dir = basename($cwd);

my $changelog = changelog_parse();

my $pkg = mustsetvar($changelog->{source}, _g('source package'));
my $version = mustsetvar($changelog->{version}, _g('source version'));
my ($ok, $error) = version_check($version);
error($error) unless $ok;

my $distribution = mustsetvar($changelog->{distribution}, _g('source distribution'));

my $maintainer;
if ($changedby) {
    $maintainer = $changedby;
} elsif ($maint) {
    $maintainer = $maint;
} else {
    $maintainer = mustsetvar($changelog->{maintainer}, _g('source changed by'));
}

open my $arch_env, '-|', 'dpkg-architecture', "-a$targetarch",
    "-t$targetgnusystem", '-f' or subprocerr('dpkg-architecture');
while ($_ = <$arch_env>) {
    chomp;
    my ($key, $value) = split /=/, $_, 2;
    $ENV{$key} = $value;
}
close $arch_env or subprocerr('dpkg-architecture');

my $arch;
if (build_sourceonly) {
    $arch = 'source';
} elsif (build_binaryindep) {
    $arch = 'all';
} else {
    $arch = mustsetvar($ENV{DEB_HOST_ARCH}, _g('host architecture'));
}

if (!defined $signcommand &&
    (($ENV{GNUPGHOME} && -e $ENV{GNUPGHOME}) ||
     ($ENV{HOME} && -e "$ENV{HOME}/.gnupg"))) {
    if (find_command('gpg2')) {
        $signcommand = 'gpg2';
    } elsif (find_command('gpg')) {
        $signcommand = 'gpg';
    }
}

if (not $signcommand) {
    $signsource = 0;
    $signchanges = 0;
} elsif ($signforce) {
    $signsource = 1;
    $signchanges = 1;
} elsif (($signsource or $signchanges) and $distribution eq 'UNRELEASED') {
    $signreleased = 0;
    $signsource = 0;
    $signchanges = 0;
}

# Preparation of environment stops here

(my $sversion = $version) =~ s/^\d+://;

my $pv = "${pkg}_$sversion";
my $pva = "${pkg}_${sversion}_$arch";

if (not -x 'debian/rules') {
    warning(_g('debian/rules is not executable; fixing that'));
    chmod(0755, 'debian/rules'); # No checks of failures, non fatal
}

unless ($call_target) {
    chdir('..') or syserr('chdir ..');
    withecho('dpkg-source', @source_opts, '--before-build', $dir);
    chdir($dir) or syserr("chdir $dir");
}

if ($checkbuilddep) {
    if ($admindir) {
	push @checkbuilddep_opts, "--admindir=$admindir";
    }

    system('dpkg-checkbuilddeps', @checkbuilddep_opts);
    if (not WIFEXITED($?)) {
        subprocerr('dpkg-checkbuilddeps');
    } elsif (WEXITSTATUS($?)) {
	warning(_g('build dependencies/conflicts unsatisfied; aborting'));
	warning(_g('(Use -d flag to override.)'));

	if (build_sourceonly) {
	    warning(_g('this is currently a non-fatal warning with -S, but ' .
	               'will probably become fatal in the future'));
	} else {
	    exit 3;
	}
    }
}

if ($call_target) {
    if ($call_target_as_root or
        $call_target =~ /^(clean|binary(|-arch|-indep))$/)
    {
        withecho(@rootcommand, @debian_rules, $call_target);
    } else {
        withecho(@debian_rules, $call_target);
    }
    exit 0;
}

unless ($noclean) {
    withecho(@rootcommand, @debian_rules, 'clean');
}
unless (build_binaryonly) {
    warning(_g('building a source package without cleaning up as you asked; ' .
               'it might contain undesired files')) if $noclean;
    chdir('..') or syserr('chdir ..');
    withecho('dpkg-source', @source_opts, '-b', $dir);
    chdir($dir) or syserr("chdir $dir");
}

if ($buildtarget ne 'build' and scalar(@debian_rules) == 1) {
    # Verify that build-{arch,indep} are supported. If not, fallback to build.
    # This is a temporary measure to not break too many packages on a flag day.
    my $pid = spawn(exec => [ 'make', '-f', @debian_rules, '-qn', $buildtarget ],
                    from_file => '/dev/null', to_file => '/dev/null',
                    error_to_file => '/dev/null');
    my $cmdline = "make -f @debian_rules -qn $buildtarget";
    wait_child($pid, nocheck => 1, cmdline => $cmdline);
    my $exitcode = WEXITSTATUS($?);
    subprocerr($cmdline) unless WIFEXITED($?);
    if ($exitcode == 2) {
        warning(_g("%s must be updated to support the 'build-arch' and " .
                   "'build-indep' targets (at least '%s' seems to be " .
                   'missing)'), "@debian_rules", $buildtarget);
        $buildtarget = 'build';
    }
}

unless (build_sourceonly) {
    withecho(@debian_rules, $buildtarget);
    withecho(@rootcommand, @debian_rules, $binarytarget);
}
if ($usepause &&
    ($signchanges || (!build_binaryonly && $signsource))) {
    print _g("Press the return key to start signing process\n");
    getc();
}

my $signerrors;
unless (build_binaryonly) {
    if ($signsource && signfile("$pv.dsc")) {
	$signerrors = _g('failed to sign .dsc and .changes file');
	$signchanges = 0;
    }
}

if (defined($maint)) { push @changes_opts, "-m$maint" }
if (defined($changedby)) { push @changes_opts, "-e$changedby" }
if (defined($since)) { push @changes_opts, "-v$since" }
if (defined($desc)) { push @changes_opts, "-C$desc" }

my $chg = "../$pva.changes";
print { *STDERR } " dpkg-genchanges @changes_opts >$chg\n";
open my $changes_fh, '-|', 'dpkg-genchanges', @changes_opts
    or subprocerr('dpkg-genchanges');

open my $out_fh, '>', $chg or syserr(_g('write changes file'));

my $infiles = my $files = '';
while ($_ = <$changes_fh>) {
    print { $out_fh } $_ or syserr(_g('write changes file'));
    chomp;

    if (/^Files:/i) {
	$infiles = 1;
    } elsif ($infiles && /^\s+(.*)$/) {
	$files .= " $1 ";
    } elsif ($infiles && /^\S/) {
	$infiles = 0;
    }
}

close $changes_fh or subprocerr(_g('dpkg-genchanges'));
close $out_fh or syserr(_g('write changes file'));

my $srcmsg;
sub fileomitted($) { return $files !~ /$_[0]/ }
my $ext = compression_get_file_extension_regex();
if (fileomitted '\.deb') {
    # source only upload
    if (fileomitted "\.diff\.$ext" and fileomitted "\.debian\.tar\.$ext") {
	$srcmsg = _g('source only upload: Debian-native package');
    } elsif (fileomitted "\.orig\.tar\.$ext") {
	$srcmsg = _g('source only, diff-only upload (original source NOT included)');
    } else {
	$srcmsg = _g('source only upload (original source is included)');
    }
} else {
    $srcmsg = _g('full upload (original source is included)');
    if (fileomitted '\.dsc') {
	$srcmsg = _g('binary only upload (no source included)');
    } elsif (fileomitted "\.diff\.$ext" and fileomitted "\.debian\.tar\.$ext") {
	$srcmsg = _g('full upload; Debian-native package (full source is included)');
    } elsif (fileomitted "\.orig\.tar\.$ext") {
	$srcmsg = _g('binary and diff upload (original source NOT included)');
    } else {
	$srcmsg = _g('full upload (original source is included)');
    }
}

if ($signchanges && signfile("$pva.changes")) {
    $signerrors = _g('failed to sign .changes file');
}

if ($cleansource) {
    withecho(@rootcommand, @debian_rules, 'clean');
}
chdir('..') or syserr('chdir ..');
withecho('dpkg-source', @source_opts, '--after-build', $dir);
chdir($dir) or syserr("chdir $dir");

if (not $signreleased) {
    warning(_g('not signing UNRELEASED build; use --force-sign to override'));
}
print "$Dpkg::PROGNAME: $srcmsg\n";
if ($signerrors) {
    warning($signerrors);
    exit 1;
}

sub mustsetvar {
    my ($var, $text) = @_;

    error(_g('unable to determine %s'), $text)
	unless defined($var);

    print "$Dpkg::PROGNAME: $text $var\n";
    return $var;
}

sub withecho {
    print { *STDERR } " @_\n";
    system(@_)
	and subprocerr("@_");
}

sub signfile {
    my ($file) = @_;
    print { *STDERR } " signfile $file\n";
    my $qfile = quotemeta($file);

    system("(cat ../$qfile ; echo '') | " .
           "$signcommand --utf8-strings --local-user " .
           quotemeta($signkey || $maintainer) .
           " --clearsign --armor --textmode  > ../$qfile.asc");
    my $status = $?;
    unless ($status) {
	system('mv', '--', "../$file.asc", "../$file")
	    and subprocerr('mv');
    } else {
	system('rm', '-f', "../$file.asc")
	    and subprocerr('rm -f');
    }
    print "\n";
    return $status
}
