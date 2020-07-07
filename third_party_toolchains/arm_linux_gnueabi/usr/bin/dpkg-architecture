#!/usr/bin/perl
#
# dpkg-architecture
#
# Copyright © 1999-2001 Marcus Brinkmann <brinkmd@debian.org>
# Copyright © 2004-2005 Scott James Remnant <scott@netsplit.com>,
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

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Arch qw(get_raw_build_arch get_raw_host_arch get_gcc_host_gnu_type
                  debarch_to_cpuattrs
                  get_valid_arches debarch_eq debarch_is debarch_to_debtriplet
                  debarch_to_gnutriplet gnutriplet_to_debarch
                  debarch_to_multiarch);

textdomain('dpkg-dev');

sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    printf _g('
This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf _g(
'Usage: %s [<option>...] [<command>]')
    . "\n\n" . _g(
'Options:
  -a<debian-arch>    set current Debian architecture.
  -t<gnu-system>     set current GNU system type.
  -L                 list valid architectures.
  -f                 force flag (override variables set in environment).')
    . "\n\n" . _g(
'Commands:
  -l                 list variables (default).
  -e<debian-arch>    compare with current Debian architecture.
  -i<arch-alias>     check if current Debian architecture is <arch-alias>.
  -q<variable>       prints only the value of <variable>.
  -s                 print command to set environment variables.
  -u                 print command to unset environment variables.
  -c <command>       set environment and run the command in it.
  -?, --help         show this help message.
      --version      show the version.
'), $Dpkg::PROGNAME;
}

sub list_arches()
{
    foreach my $arch (get_valid_arches()) {
	print "$arch\n";
    }
}

use constant {
    DEB_NONE => 0,
    DEB_BUILD => 1,
    DEB_HOST => 2,
    DEB_ARCH_INFO => 4,
    DEB_ARCH_ATTR => 8,
    DEB_MULTIARCH => 16,
    DEB_GNU_INFO => 32,
};

use constant DEB_ALL => DEB_BUILD | DEB_HOST | DEB_ARCH_INFO | DEB_ARCH_ATTR |
                        DEB_MULTIARCH | DEB_GNU_INFO;

my %arch_vars = (
    DEB_BUILD_ARCH => DEB_BUILD,
    DEB_BUILD_ARCH_OS => DEB_BUILD | DEB_ARCH_INFO,
    DEB_BUILD_ARCH_CPU => DEB_BUILD | DEB_ARCH_INFO,
    DEB_BUILD_ARCH_BITS => DEB_BUILD | DEB_ARCH_ATTR,
    DEB_BUILD_ARCH_ENDIAN => DEB_BUILD | DEB_ARCH_ATTR,
    DEB_BUILD_MULTIARCH => DEB_BUILD | DEB_MULTIARCH,
    DEB_BUILD_GNU_CPU => DEB_BUILD | DEB_GNU_INFO,
    DEB_BUILD_GNU_SYSTEM => DEB_BUILD | DEB_GNU_INFO,
    DEB_BUILD_GNU_TYPE => DEB_BUILD | DEB_GNU_INFO,
    DEB_HOST_ARCH => DEB_HOST,
    DEB_HOST_ARCH_OS => DEB_HOST | DEB_ARCH_INFO,
    DEB_HOST_ARCH_CPU => DEB_HOST | DEB_ARCH_INFO,
    DEB_HOST_ARCH_BITS => DEB_HOST | DEB_ARCH_ATTR,
    DEB_HOST_ARCH_ENDIAN => DEB_HOST | DEB_ARCH_ATTR,
    DEB_HOST_MULTIARCH => DEB_HOST | DEB_MULTIARCH,
    DEB_HOST_GNU_CPU => DEB_HOST | DEB_GNU_INFO,
    DEB_HOST_GNU_SYSTEM => DEB_HOST | DEB_GNU_INFO,
    DEB_HOST_GNU_TYPE => DEB_HOST | DEB_GNU_INFO,
);

my $req_vars = DEB_ALL;
my $req_host_arch = '';
my $req_host_gnu_type = '';
my $req_eq_arch = '';
my $req_is_arch = '';
my $req_variable_to_print;
my $action = 'l';
my $force = 0;

sub action_needs($) {
  my ($bits) = @_;
  return (($req_vars & $bits) == $bits);
}

while (@ARGV) {
    $_=shift(@ARGV);
    if (m/^-a/) {
	$req_host_arch = "$'";
    } elsif (m/^-t/) {
	$req_host_gnu_type = "$'";
    } elsif (m/^-e/) {
	$req_eq_arch = "$'";
	$req_vars = $arch_vars{DEB_HOST_ARCH};
	$action = 'e';
    } elsif (m/^-i/) {
	$req_is_arch = "$'";
	$req_vars = $arch_vars{DEB_HOST_ARCH};
	$action = 'i';
    } elsif (m/^-u$/) {
	$req_vars = DEB_NONE;
	$action = 'u';
    } elsif (m/^-[ls]$/) {
	$action = $_;
	$action =~ s/^-//;
    } elsif (m/^-f$/) {
        $force=1;
    } elsif (m/^-q/) {
	my $varname = "$'";
	error(_g('%s is not a supported variable name'), $varname)
	    unless (exists $arch_vars{$varname});
	$req_variable_to_print = "$varname";
	$req_vars = $arch_vars{$varname};
        $action = 'q';
    } elsif (m/^-c$/) {
       $action = 'c';
       last;
    } elsif (m/^-L$/) {
        list_arches();
        exit unless @ARGV;
    } elsif (m/^-(\?|-help)$/) {
        usage();
       exit 0;
    } elsif (m/^--version$/) {
        version();
       exit 0;
    } else {
	usageerr(_g("unknown option \`%s'"), $_);
    }
}

my %v;
my $abi;

#
# Set build variables
#

$v{DEB_BUILD_ARCH} = get_raw_build_arch()
    if (action_needs(DEB_BUILD));
($abi, $v{DEB_BUILD_ARCH_OS}, $v{DEB_BUILD_ARCH_CPU}) = debarch_to_debtriplet($v{DEB_BUILD_ARCH})
    if (action_needs(DEB_BUILD | DEB_ARCH_INFO));
($v{DEB_BUILD_ARCH_BITS}, $v{DEB_BUILD_ARCH_ENDIAN}) = debarch_to_cpuattrs($v{DEB_BUILD_ARCH})
    if (action_needs(DEB_BUILD | DEB_ARCH_ATTR));

$v{DEB_BUILD_MULTIARCH} = debarch_to_multiarch($v{DEB_BUILD_ARCH})
    if (action_needs(DEB_BUILD | DEB_MULTIARCH));

if (action_needs(DEB_BUILD | DEB_GNU_INFO)) {
  $v{DEB_BUILD_GNU_TYPE} = debarch_to_gnutriplet($v{DEB_BUILD_ARCH});
  ($v{DEB_BUILD_GNU_CPU}, $v{DEB_BUILD_GNU_SYSTEM}) = split(/-/, $v{DEB_BUILD_GNU_TYPE}, 2);
}

#
# Set host variables
#

# First perform some sanity checks on the host arguments passed.

if ($req_host_arch ne '' && $req_host_gnu_type eq '') {
    $req_host_gnu_type = debarch_to_gnutriplet($req_host_arch);
    error(_g('unknown Debian architecture %s, you must specify ' .
             'GNU system type, too'), $req_host_arch)
        unless defined $req_host_gnu_type;
}

if ($req_host_gnu_type ne '' && $req_host_arch eq '') {
    $req_host_arch = gnutriplet_to_debarch($req_host_gnu_type);
    error(_g('unknown GNU system type %s, you must specify ' .
             'Debian architecture, too'), $req_host_gnu_type)
        unless defined $req_host_arch;
}

if ($req_host_gnu_type ne '' && $req_host_arch ne '') {
    my $dfl_host_gnu_type = debarch_to_gnutriplet($req_host_arch);
    error(_g('unknown default GNU system type for Debian architecture %s'),
          $req_host_arch)
	unless defined $dfl_host_gnu_type;
    warning(_g('default GNU system type %s for Debian arch %s does not ' .
               'match specified GNU system type %s'), $dfl_host_gnu_type,
            $req_host_arch, $req_host_gnu_type)
        if $dfl_host_gnu_type ne $req_host_gnu_type;
}

# Proceed to compute the host variables if needed.

if (action_needs(DEB_HOST)) {
    if ($req_host_arch eq '') {
        $v{DEB_HOST_ARCH} = get_raw_host_arch();
    } else {
        $v{DEB_HOST_ARCH} = $req_host_arch;
    }
}
($abi, $v{DEB_HOST_ARCH_OS}, $v{DEB_HOST_ARCH_CPU}) = debarch_to_debtriplet($v{DEB_HOST_ARCH})
    if (action_needs(DEB_HOST | DEB_ARCH_INFO));
($v{DEB_HOST_ARCH_BITS}, $v{DEB_HOST_ARCH_ENDIAN}) = debarch_to_cpuattrs($v{DEB_HOST_ARCH})
    if (action_needs(DEB_HOST | DEB_ARCH_ATTR));

$v{DEB_HOST_MULTIARCH} = debarch_to_multiarch($v{DEB_HOST_ARCH})
    if (action_needs(DEB_HOST | DEB_MULTIARCH));

if (action_needs(DEB_HOST | DEB_GNU_INFO)) {
    if ($req_host_gnu_type eq '') {
        $v{DEB_HOST_GNU_TYPE} = debarch_to_gnutriplet($v{DEB_HOST_ARCH});
    } else {
        $v{DEB_HOST_GNU_TYPE} = $req_host_gnu_type;
    }
    ($v{DEB_HOST_GNU_CPU}, $v{DEB_HOST_GNU_SYSTEM}) = split(/-/, $v{DEB_HOST_GNU_TYPE}, 2);

    my $gcc = get_gcc_host_gnu_type();

    warning(_g('specified GNU system type %s does not match gcc system ' .
               'type %s, try setting a correct CC environment variable'),
            $v{DEB_HOST_GNU_TYPE}, $gcc)
        if ($gcc ne '') && ($gcc ne $v{DEB_HOST_GNU_TYPE});
}


for my $k (keys %arch_vars) {
    $v{$k} = $ENV{$k} if (defined ($ENV{$k}) && !$force);
}

if ($action eq 'l') {
    foreach my $k (sort keys %arch_vars) {
	print "$k=$v{$k}\n";
    }
} elsif ($action eq 's') {
    foreach my $k (sort keys %arch_vars) {
	print "$k=$v{$k}; ";
    }
    print 'export ' . join(' ', sort keys %arch_vars) . "\n";
} elsif ($action eq 'u') {
    print 'unset ' . join(' ', sort keys %arch_vars) . "\n";
} elsif ($action eq 'e') {
    exit !debarch_eq($v{DEB_HOST_ARCH}, $req_eq_arch);
} elsif ($action eq 'i') {
    exit !debarch_is($v{DEB_HOST_ARCH}, $req_is_arch);
} elsif ($action eq 'c') {
    @ENV{keys %v} = values %v;
    exec @ARGV;
} elsif ($action eq 'q') {
    print "$v{$req_variable_to_print}\n";
}
