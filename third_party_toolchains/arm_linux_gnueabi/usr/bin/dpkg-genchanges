#!/usr/bin/perl
#
# dpkg-genchanges
#
# Copyright © 1996 Ian Jackson
# Copyright © 2000,2001 Wichert Akkerman
# Copyright © 2006-2012 Guillem Jover <guillem@debian.org>
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

use Encode;
use POSIX qw(:errno_h);
use Dpkg ();
use Dpkg::Gettext;
use Dpkg::Util qw(:list);
use Dpkg::File;
use Dpkg::Checksums;
use Dpkg::ErrorHandling;
use Dpkg::BuildProfiles qw(get_build_profiles);
use Dpkg::Arch qw(get_host_arch debarch_eq debarch_is);
use Dpkg::Compression;
use Dpkg::Control::Info;
use Dpkg::Control::Fields;
use Dpkg::Control;
use Dpkg::Substvars;
use Dpkg::Vars;
use Dpkg::Changelog::Parse;
use Dpkg::Version;

textdomain('dpkg-dev');

my $controlfile = 'debian/control';
my $changelogfile = 'debian/changelog';
my $changelogformat;
my $fileslistfile = 'debian/files';
my $uploadfilesdir = '..';
my $sourcestyle = 'i';
my $quiet = 0;
my $host_arch = get_host_arch();
my $changes_format = '1.8';

my %f2p;           # - file to package map
my %p2f;           # - package to file map, has entries for "packagename"
my %pa2f;          # - likewise, has entries for "packagename architecture"
my %p2ver;         # - package to version map
my %p2arch;        # - package to arch map
my %f2sec;         # - file to section map
my %f2seccf;       # - likewise, from control file
my %f2pri;         # - file to priority map
my %f2pricf;       # - likewise, from control file
my %sourcedefault; # - default values as taken from source (used for Section,
                   #   Priority and Maintainer)

my @descriptions;
my @fileslistfiles;

my $checksums = Dpkg::Checksums->new();
my %remove;        # - fields to remove
my %override;
my %archadded;
my @archvalues;
my $dsc;
my $changesdescription;
my $forcemaint;
my $forcechangedby;
my $since;

my $substvars_loaded = 0;
my $substvars = Dpkg::Substvars->new();
$substvars->set('Format', $changes_format);

use constant SOURCE     => 1;
use constant ARCH_DEP   => 2;
use constant ARCH_INDEP => 4;
use constant BIN        => ARCH_DEP | ARCH_INDEP;
use constant ALL        => BIN | SOURCE;
my $include = ALL;

sub is_sourceonly() { return $include == SOURCE; }
sub is_binaryonly() { return !($include & SOURCE); }
sub binary_opt() { return (($include == BIN) ? '-b' :
			   (($include == ARCH_DEP) ? '-B' :
			    (($include == ARCH_INDEP) ? '-A' :
			     internerr("binary_opt called with include=$include"))));
}

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
"Options:
  -b                       binary-only build - no source files.
  -B                       arch-specific - no source or arch-indep files.
  -A                       only arch-indep - no source or arch-specific files.
  -S                       source-only upload.
  -c<control-file>         get control info from this file.
  -l<changelog-file>       get per-version info from this file.
  -f<files-list-file>      get .deb files list from this file.
  -v<since-version>        include all changes later than version.
  -C<changes-description>  use change description from this file.
  -m<maintainer>           override control's maintainer value.
  -e<maintainer>           override changelog's maintainer value.
  -u<upload-files-dir>     directory with files (default is '..').
  -si (default)            src includes orig if new upstream.
  -sa                      source includes orig src.
  -sd                      source is diff and .dsc only.
  -q                       quiet - no informational messages on stderr.
  -F<changelog-format>     force changelog format.
  -V<name>=<value>         set a substitution variable.
  -T<substvars-file>       read variables here, not debian/substvars.
  -D<field>=<value>        override or add a field and value.
  -U<field>                remove a field.
  -?, --help               show this help message.
      --version            show the version.
"), $Dpkg::PROGNAME;
}


while (@ARGV) {
    $_=shift(@ARGV);
    if (m/^-b$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if is_sourceonly;
	$include = BIN;
    } elsif (m/^-B$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if is_sourceonly;
	$include = ARCH_DEP;
	printf { *STDERR } _g('%s: arch-specific upload - not including arch-independent packages') . "\n", $Dpkg::PROGNAME;
    } elsif (m/^-A$/) {
	usageerr(_g('cannot combine %s and %s'), $_, '-S') if is_sourceonly;
	$include = ARCH_INDEP;
	printf { *STDERR } _g('%s: arch-indep upload - not including arch-specific packages') . "\n", $Dpkg::PROGNAME;
    } elsif (m/^-S$/) {
	usageerr(_g('cannot combine %s and %s'), binary_opt, '-S')
	    if is_binaryonly;
	$include = SOURCE;
    } elsif (m/^-s([iad])$/) {
        $sourcestyle= $1;
    } elsif (m/^-q$/) {
        $quiet= 1;
    } elsif (m/^-c(.*)$/) {
	$controlfile = $1;
    } elsif (m/^-l(.*)$/) {
	$changelogfile = $1;
    } elsif (m/^-C(.*)$/) {
	$changesdescription = $1;
    } elsif (m/^-f(.*)$/) {
	$fileslistfile = $1;
    } elsif (m/^-v(.*)$/) {
	$since = $1;
    } elsif (m/^-T(.*)$/) {
	$substvars->load($1) if -e $1;
	$substvars_loaded = 1;
    } elsif (m/^-m(.*)$/s) {
	$forcemaint = $1;
    } elsif (m/^-e(.*)$/s) {
	$forcechangedby = $1;
    } elsif (m/^-F([0-9a-z]+)$/) {
        $changelogformat = $1;
    } elsif (m/^-D([^\=:]+)[=:](.*)$/s) {
	$override{$1} = $2;
    } elsif (m/^-u(.*)$/) {
	$uploadfilesdir = $1;
    } elsif (m/^-U([^\=:]+)$/) {
        $remove{$1} = 1;
    } elsif (m/^-V(\w[-:0-9A-Za-z]*)[=:](.*)$/s) {
	$substvars->set($1, $2);
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

# Retrieve info from the current changelog entry
my %options = (file => $changelogfile);
$options{changelogformat} = $changelogformat if $changelogformat;
$options{since} = $since if defined($since);
my $changelog = changelog_parse(%options);
# Change options to retrieve info of the former changelog entry
delete $options{since};
$options{count} = 1;
$options{offset} = 1;
my $prev_changelog = changelog_parse(%options);
# Other initializations
my $control = Dpkg::Control::Info->new($controlfile);
my $fields = Dpkg::Control->new(type => CTRL_FILE_CHANGES);

my $sourceversion = $changelog->{'Binary-Only'} ?
                    $prev_changelog->{'Version'} : $changelog->{'Version'};
my $binaryversion = $changelog->{'Version'};

$substvars->set_version_substvars($sourceversion, $binaryversion);
$substvars->set_arch_substvars();
$substvars->load('debian/substvars') if -e 'debian/substvars' and not $substvars_loaded;

if (defined($prev_changelog) and
    version_compare_relation($changelog->{'Version'}, REL_LT,
                             $prev_changelog->{'Version'}))
{
    warning(_g('the current version (%s) is earlier than the previous one (%s)'),
	$changelog->{'Version'}, $prev_changelog->{'Version'})
        # ~bpo and ~vola are backports and have lower version number by definition
        unless $changelog->{'Version'} =~ /~(?:bpo|vola)/;
}

if (not is_sourceonly) {
    open(my $fileslist_fh, '<', $fileslistfile)
        or syserr(_g('cannot read files list file'));
    while(<$fileslist_fh>) {
	if (m/^(([-+.0-9a-z]+)_([^_]+)_([-\w]+)\.u?deb) (\S+) (\S+)$/) {
	    warning(_g('duplicate files list entry for package %s (line %d)'),
	            $2, $.) if defined $p2f{"$2 $4"};
	    $f2p{$1}= $2;
	    $pa2f{"$2 $4"}= $1;
	    $p2f{$2} ||= [];
	    push @{$p2f{$2}}, $1;
	    $p2ver{$2}= $3;

	    warning(_g('duplicate files list entry for file %s (line %d)'),
	            $1, $.) if defined $f2sec{$1};
	    $f2sec{$1}= $5;
	    $f2pri{$1}= $6;
	    push(@archvalues, $4) if $4 and not $archadded{$4}++;
	    push(@fileslistfiles,$1);
	} elsif (m/^([-+.0-9a-z]+_[^_]+_([-\w]+)\.[a-z0-9.]+) (\S+) (\S+)$/) {
	    # A non-deb package
	    $f2sec{$1}= $3;
	    $f2pri{$1}= $4;
	    push(@archvalues, $2) if $2 and not $archadded{$2}++;
	    push(@fileslistfiles,$1);
	} elsif (m/^([-+.,_0-9a-zA-Z]+) (\S+) (\S+)$/) {
	    warning(_g('duplicate files list entry for file %s (line %d)'),
	            $1, $.) if defined $f2sec{$1};
	    $f2sec{$1}= $2;
	    $f2pri{$1}= $3;
	    push(@fileslistfiles,$1);
	} else {
	    error(_g('badly formed line in files list file, line %d'), $.);
	}
    }
    close($fileslist_fh);
}

# Scan control info of source package
my $src_fields = $control->get_source();
foreach (keys %{$src_fields}) {
    my $v = $src_fields->{$_};
    if (m/^Source$/) {
	set_source_package($v);
    } elsif (m/^Section$|^Priority$/i) {
	$sourcedefault{$_} = $v;
    } else {
        field_transfer_single($src_fields, $fields);
    }
}

# Scan control info of all binary packages
foreach my $pkg ($control->get_packages()) {
    my $p = $pkg->{'Package'};
    my $a = $pkg->{'Architecture'} || '';
    my $d = $pkg->{'Description'} || 'no description available';
    $d = $1 if $d =~ /^(.*)\n/;
    my $pkg_type = $pkg->{'Package-Type'} ||
                   $pkg->get_custom_field('Package-Type') || 'deb';

    my @f; # List of files for this binary package
    push @f, @{$p2f{$p}} if defined $p2f{$p};

    # Add description of all binary packages
    my $desc = encode_utf8(sprintf('%-10s - %-.65s', $p, decode_utf8($d)));
    $desc .= ' (udeb)' if $pkg_type eq 'udeb';
    push @descriptions, $desc;

    if (not defined($p2f{$p})) {
	# No files for this package... warn if it's unexpected
	if ((debarch_eq('all', $a) and ($include & ARCH_INDEP)) ||
	    ((any { debarch_is($host_arch, $_) } split /\s+/, $a)
		  and ($include & ARCH_DEP))) {
	    warning(_g('package %s in control file but not in files list'),
		    $p);
	}
	next; # and skip it
    }

    $p2arch{$p} = $a;

    foreach (keys %{$pkg}) {
	my $v = $pkg->{$_};

	if (m/^Section$/) {
	    $f2seccf{$_} = $v foreach (@f);
	} elsif (m/^Priority$/) {
	    $f2pricf{$_} = $v foreach (@f);
	} elsif (m/^Architecture$/) {
	    if ((any { debarch_is($host_arch, $_) } split /\s+/, $v)
		and ($include & ARCH_DEP)) {
		$v = $host_arch;
	    } elsif (!debarch_eq('all', $v)) {
		$v = '';
	    }
	    push(@archvalues, $v) if $v and not $archadded{$v}++;
        } elsif (m/^Description$/) {
            # Description in changes is computed, do not copy this field
	} else {
            field_transfer_single($pkg, $fields);
	}
    }
}

# Scan fields of dpkg-parsechangelog
foreach (keys %{$changelog}) {
    my $v = $changelog->{$_};
    if (m/^Source$/i) {
	set_source_package($v);
    } elsif (m/^Maintainer$/i) {
	$fields->{'Changed-By'} = $v;
    } else {
        field_transfer_single($changelog, $fields);
    }
}

if ($changesdescription) {
    open(my $changes_fh, '<', $changesdescription)
        or syserr(_g('read changesdescription'));
    $fields->{'Changes'} = "\n" . file_slurp($changes_fh);
    close($changes_fh);
}

for my $pa (keys %pa2f) {
    my ($pp, $aa) = (split / /, $pa);

    warning(_g('package %s listed in files list but not in control info'), $pp)
        unless defined $control->get_pkg_by_name($pp);
}

for my $p (keys %p2f) {
    my @f = @{$p2f{$p}};

    foreach my $f (@f) {
	my $sec = $f2seccf{$f};
	$sec ||= $sourcedefault{'Section'};
	if (!defined($sec)) {
	    $sec = '-';
	    warning(_g("missing Section for binary package %s; using '-'"), $p);
	}
	if ($sec ne $f2sec{$f}) {
	    error(_g('package %s has section %s in control file but %s in ' .
	             'files list'), $p, $sec, $f2sec{$f});
	}

	my $pri = $f2pricf{$f};
	$pri ||= $sourcedefault{'Priority'};
	if (!defined($pri)) {
	    $pri = '-';
	    warning(_g("missing Priority for binary package %s; using '-'"), $p);
	}
	if ($pri ne $f2pri{$f}) {
	    error(_g('package %s has priority %s in control file but %s in ' .
	             'files list'), $p, $pri, $f2pri{$f});
	}
    }
}

my $origsrcmsg;

if (!is_binaryonly) {
    my $sec = $sourcedefault{'Section'};
    if (!defined($sec)) {
	$sec = '-';
	warning(_g('missing Section for source files'));
    }
    my $pri = $sourcedefault{'Priority'};
    if (!defined($pri)) {
	$pri = '-';
	warning(_g('missing Priority for source files'));
    }

    my $spackage = get_source_package();
    (my $sversion = $substvars->get('source:Version')) =~ s/^\d+://;
    $dsc= "$uploadfilesdir/${spackage}_${sversion}.dsc";

    my $dsc_fields = Dpkg::Control->new(type => CTRL_PKG_SRC);
    $dsc_fields->load($dsc) or error(_g('%s is empty', $dsc));
    $checksums->add_from_file($dsc, key => "$spackage\_$sversion.dsc");
    $checksums->add_from_control($dsc_fields, use_files_for_md5 => 1);

    for my $f ($checksums->get_files()) {
	$f2sec{$f} = $sec;
	$f2pri{$f} = $pri;
    }

    # Compare upstream version to previous upstream version to decide if
    # the .orig tarballs must be included
    my $include_tarball;
    if (defined($prev_changelog)) {
	my $cur = Dpkg::Version->new($changelog->{'Version'});
	my $prev = Dpkg::Version->new($prev_changelog->{'Version'});
	$include_tarball = ($cur->version() ne $prev->version()) ? 1 : 0;
    } else {
	# No previous entry means first upload, tarball required
	$include_tarball = 1;
    }

    my $ext = compression_get_file_extension_regex();
    if ((($sourcestyle =~ m/i/ && !$include_tarball) ||
	 $sourcestyle =~ m/d/) &&
	any { m/\.(debian\.tar|diff)\.$ext$/ } $checksums->get_files())
    {
	$origsrcmsg= _g('not including original source code in upload');
	foreach my $f (grep { m/\.orig(-.+)?\.tar\.$ext$/ } $checksums->get_files()) {
	    $checksums->remove_file($f);
	}
    } else {
	if ($sourcestyle =~ m/d/ &&
	    none { m/\.(debian\.tar|diff)\.$ext$/ } $checksums->get_files()) {
	    warning(_g('ignoring -sd option for native Debian package'));
	}
        $origsrcmsg= _g('including full source code in upload');
    }
} else {
    $origsrcmsg= _g('binary-only upload - not including any source code');
}

print { *STDERR } "$Dpkg::PROGNAME: $origsrcmsg\n"
    or syserr(_g('write original source message')) unless $quiet;

$fields->{'Format'} = $substvars->get('Format');

if (!defined($fields->{'Date'})) {
    chomp(my $date822 = `date -R`);
    subprocerr('date -R') if $?;
    $fields->{'Date'}= $date822;
}

$fields->{'Binary'} = join(' ', map { $_->{'Package'} } $control->get_packages());
# Avoid overly long line by splitting over multiple lines
if (length($fields->{'Binary'}) > 980) {
    $fields->{'Binary'} =~ s/(.{0,980}) /$1\n/g;
}

unshift(@archvalues,'source') unless is_binaryonly;
@archvalues = ('all') if $include == ARCH_INDEP;
@archvalues = grep {!debarch_eq('all',$_)} @archvalues
    unless $include & ARCH_INDEP;
$fields->{'Architecture'} = join(' ',@archvalues);

$fields->{'Built-For-Profiles'} = join ' ', get_build_profiles();

$fields->{'Description'} = "\n" . join("\n", sort @descriptions);

$fields->{'Files'} = '';

my %filedone;

for my $f ($checksums->get_files(), @fileslistfiles) {
    if (defined $f2p{$f}) {
        my $arch_all = debarch_eq('all', $p2arch{$f2p{$f}});

        next if ($include == ARCH_DEP and $arch_all);
        next if ($include == ARCH_INDEP and not $arch_all);
    }
    next if $filedone{$f}++;
    my $uf = "$uploadfilesdir/$f";
    $checksums->add_from_file($uf, key => $f);
    $fields->{'Files'} .= "\n" . $checksums->get_checksum($f, 'md5') .
			  ' ' . $checksums->get_size($f) .
			  " $f2sec{$f} $f2pri{$f} $f";
}
$checksums->export_to_control($fields);
# redundant with the Files field
delete $fields->{'Checksums-Md5'};

$fields->{'Source'} = get_source_package();
if ($fields->{'Version'} ne $substvars->get('source:Version')) {
    $fields->{'Source'} .= ' (' . $substvars->get('source:Version') . ')';
}

$fields->{'Maintainer'} = $forcemaint if defined($forcemaint);
$fields->{'Changed-By'} = $forcechangedby if defined($forcechangedby);

for my $f (qw(Version Distribution Maintainer Changes)) {
    error(_g('missing information for critical output field %s'), $f)
        unless defined $fields->{$f};
}

for my $f (qw(Urgency)) {
    warning(_g('missing information for output field %s'), $f)
        unless defined $fields->{$f};
}

for my $f (keys %override) {
    $fields->{$f} = $override{$f};
}
for my $f (keys %remove) {
    delete $fields->{$f};
}

# Note: do not perform substitution of variables, one of the reasons is that
# they could interfere with field values, for example the Changes field.
$fields->output(\*STDOUT);
