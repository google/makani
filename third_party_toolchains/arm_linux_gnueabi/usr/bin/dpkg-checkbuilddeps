#!/usr/bin/perl
#
# dpkg-checkbuilddeps
#
# Copyright © 2001 Joey Hess <joeyh@debian.org>
# Copyright © 2006-2009,2011-2012 Guillem Jover <guillem@debian.org>
# Copyright © 2007-2011 Raphael Hertzog <hertzog@debian.org>
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

use Getopt::Long qw(:config posix_default bundling no_ignorecase);

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Arch qw(get_host_arch);
use Dpkg::BuildProfiles qw(get_build_profiles set_build_profiles);
use Dpkg::Deps;
use Dpkg::Control::Info;

textdomain('dpkg-dev');

sub version()
{
	printf(_g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION);
	exit(0);
}

sub usage {
	printf _g(
'Usage: %s [<option>...] [<control-file>]')
	. "\n\n" . _g(
'Options:
  -A             ignore Build-Depends-Arch and Build-Conflicts-Arch.
  -B             ignore Build-Depends-Indep and Build-Conflicts-Indep.
  -d build-deps  use given string as build dependencies instead of
                 retrieving them from control file
  -c build-conf  use given string for build conflicts instead of
                 retrieving them from control file
  -a arch        assume given host architecture
  -P profiles    assume given build profiles (comma-separated list)
  --admindir=<directory>
                 change the administrative directory.
  -?, --help     show this help message.
      --version  show the version.')
	. "\n\n" . _g(
'<control-file> is the control file to process (default: debian/control).')
	. "\n", $Dpkg::PROGNAME;
}

my $ignore_bd_arch = 0;
my $ignore_bd_indep = 0;
my ($bd_value, $bc_value);
my $bp_value;
my $host_arch = get_host_arch();
my $admindir = $Dpkg::ADMINDIR;
my @options_spec = (
    'help|?' => sub { usage(); exit(0); },
    'version' => \&version,
    'A' => \$ignore_bd_arch,
    'B' => \$ignore_bd_indep,
    'd=s' => \$bd_value,
    'c=s' => \$bc_value,
    'a=s' => \$host_arch,
    'P=s' => \$bp_value,
    'admindir=s' => \$admindir,
);

{
    local $SIG{__WARN__} = sub { usageerr($_[0]) };
    GetOptions(@options_spec);
}

# Update currently active build profiles.
set_build_profiles(split(/,/, $bp_value)) if ($bp_value);
my @build_profiles = get_build_profiles();

my $controlfile = shift || 'debian/control';

my $control = Dpkg::Control::Info->new($controlfile);
my $fields = $control->get_source();

my $facts = parse_status("$admindir/status");

unless (defined($bd_value) or defined($bc_value)) {
    my @bd_list = ('build-essential:native', $fields->{'Build-Depends'});
    push @bd_list, $fields->{'Build-Depends-Arch'} if not $ignore_bd_arch;
    push @bd_list, $fields->{'Build-Depends-Indep'} if not $ignore_bd_indep;
    $bd_value = deps_concat(@bd_list);

    my @bc_list = ($fields->{'Build-Conflicts'});
    push @bc_list, $fields->{'Build-Conflicts-Arch'} if not $ignore_bd_arch;
    push @bc_list, $fields->{'Build-Conflicts-Indep'} if not $ignore_bd_indep;
    $bc_value = deps_concat(@bc_list);
}
my (@unmet, @conflicts);

if ($bd_value) {
	push @unmet, build_depends('Build-Depends/Build-Depends-Arch/Build-Depends-Indep',
		deps_parse($bd_value, reduce_restrictions => 1, build_dep => 1,
			   build_profiles => \@build_profiles,
			   host_arch => $host_arch), $facts);
}
if ($bc_value) {
	push @conflicts, build_conflicts('Build-Conflicts/Build-Conflicts-Arch/Build-Conflicts-Indep',
		deps_parse($bc_value, reduce_restrictions => 1, build_dep => 1,
			   build_profiles => \@build_profiles, union => 1,
			   host_arch => $host_arch), $facts);
}

if (@unmet) {
	printf { *STDERR } _g('%s: Unmet build dependencies: '), $Dpkg::PROGNAME;
	print { *STDERR } join(' ', map { $_->output() } @unmet), "\n";
}
if (@conflicts) {
	printf { *STDERR } _g('%s: Build conflicts: '), $Dpkg::PROGNAME;
	print { *STDERR } join(' ', map { $_->output() } @conflicts), "\n";
}
exit 1 if @unmet || @conflicts;

# Silly little status file parser that returns a Dpkg::Deps::KnownFacts
sub parse_status {
	my $status = shift;

	my $facts = Dpkg::Deps::KnownFacts->new();
	local $/ = '';
	open(my $status_fh, '<', $status)
		or syserr(_g('cannot open %s'), $status);
	while (<$status_fh>) {
		next unless /^Status: .*ok installed$/m;

		my ($package) = /^Package: (.*)$/m;
		my ($version) = /^Version: (.*)$/m;
		my ($arch) = /^Architecture: (.*)$/m;
		my ($multiarch) = /^Multi-Arch: (.*)$/m;
		$facts->add_installed_package($package, $version, $arch,
		                              $multiarch);

		if (/^Provides: (.*)$/m) {
			my $provides = deps_parse($1, reduce_arch => 1, union => 1);
			next if not defined $provides;
			foreach (grep { $_->isa('Dpkg::Deps::Simple') }
                                 $provides->get_deps())
			{
				$facts->add_provided_package($_->{package},
                                    $_->{relation}, $_->{version},
                                    $package);
			}
		}
	}
	close $status_fh;

	return $facts;
}

# This function checks the build dependencies passed in as the first
# parameter. If they are satisfied, returns false. If they are unsatisfied,
# an list of the unsatisfied depends is returned.
#
# Additional parameters that must be passed:
# * A reference to a hash of all "ok installed" the packages on the system,
#   with the hash key being the package name, and the value being the
#   installed version.
# * A reference to a hash, where the keys are package names, and the
#   value is a true value iff some package installed on the system provides
#   that package (all installed packages provide themselves)
#
# Optionally, the architecture the package is to be built for can be passed
# in as the 4th parameter. If not set, dpkg will be queried for the build
# architecture.
sub build_depends {
	return check_line(1, @_);
}

# This function is exactly like unmet_build_depends, except it
# checks for build conflicts, and returns a list of the packages
# that are installed and are conflicted with.
sub build_conflicts {
	return check_line(0, @_);
}

# This function does all the work. The first parameter is 1 to check build
# deps, and 0 to check build conflicts.
sub check_line {
	my $build_depends=shift;
	my $fieldname=shift;
	my $dep_list=shift;
	my $facts=shift;

	my @unmet=();

	unless(defined($dep_list)) {
	    error(_g('error occurred while parsing %s'), $fieldname);
	}

	if ($build_depends) {
		$dep_list->simplify_deps($facts);
		if ($dep_list->is_empty()) {
			return ();
		} else {
			return $dep_list->get_deps();
		}
	} else { # Build-Conflicts
		my @conflicts = ();
		foreach my $dep ($dep_list->get_deps()) {
			if ($dep->get_evaluation($facts)) {
				push @conflicts, $dep;
			}
		}
		return @conflicts;
	}
}
