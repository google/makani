#!/usr/bin/perl
#
# dpkg-scanpackages
#
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

use warnings;
use strict;

use IO::Handle;
use IO::File;
use Getopt::Long qw(:config posix_default bundling no_ignorecase);

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Util qw(:list);
use Dpkg::Control;
use Dpkg::Version;
use Dpkg::Checksums;
use Dpkg::Compression::FileHandle;
use Dpkg::IPC;

textdomain('dpkg-dev');

# Do not pollute STDOUT with info messages
report_options(info_fh => \*STDERR);

my (@samemaint, @changedmaint);
my @spuriousover;
my %packages;
my %overridden;

my %options = (help            => sub { usage(); exit 0; },
	       version         => \&version,
	       type            => undef,
	       arch            => undef,
	       multiversion    => 0,
	       'extra-override'=> undef,
               medium          => undef,
	      );

my @options_spec = (
    'help|?',
    'version',
    'type|t=s',
    'arch|a=s',
    'multiversion|m!',
    'extra-override|e=s',
    'medium|M=s',
);

sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;
    exit;
}

sub usage {
    printf _g(
"Usage: %s [<option>...] <binary-path> [<override-file> [<path-prefix>]] > Packages

Options:
  -t, --type <type>        scan for <type> packages (default is 'deb').
  -a, --arch <arch>        architecture to scan for.
  -m, --multiversion       allow multiple versions of a single package.
  -e, --extra-override <file>
                           use extra override file.
  -M, --medium <medium>    add X-Medium field for dselect multicd access method
  -?, --help               show this help message.
      --version            show the version.
"), $Dpkg::PROGNAME;
}

sub load_override
{
    my $override = shift;
    my $comp_file = Dpkg::Compression::FileHandle->new(filename => $override);

    while (<$comp_file>) {
	s/\#.*//;
	s/\s+$//;
	next unless $_;

	my ($p, $priority, $section, $maintainer) = split(/\s+/, $_, 4);

	if (not defined($packages{$p})) {
	    push(@spuriousover, $p);
	    next;
	}

	for my $package (@{$packages{$p}}) {
	    if ($maintainer) {
		if ($maintainer =~ m/(.+?)\s*=\>\s*(.+)/) {
		    my $oldmaint = $1;
		    my $newmaint = $2;
		    my $debmaint = $$package{Maintainer};
		    if (none { $debmaint eq $_ } split m{\s*//\s*}, $oldmaint) {
			push(@changedmaint,
			     sprintf(_g('  %s (package says %s, not %s)'),
			             $p, $$package{Maintainer}, $oldmaint));
		    } else {
			$$package{Maintainer} = $newmaint;
		    }
		} elsif ($$package{Maintainer} eq $maintainer) {
		    push(@samemaint, "  $p ($maintainer)");
		} else {
		    warning(_g('unconditional maintainer override for %s'), $p);
		    $$package{Maintainer} = $maintainer;
		}
	    }
	    $$package{Priority} = $priority;
	    $$package{Section} = $section;
	}
	$overridden{$p} = 1;
    }

    close($comp_file);
}

sub load_override_extra
{
    my $extra_override = shift;
    my $comp_file = Dpkg::Compression::FileHandle->new(filename => $extra_override);

    while (<$comp_file>) {
	s/\#.*//;
	s/\s+$//;
	next unless $_;

	my ($p, $field, $value) = split(/\s+/, $_, 3);

	next unless defined($packages{$p});

	for my $package (@{$packages{$p}}) {
	    $$package{$field} = $value;
	}
    }

    close($comp_file);
}

{
    local $SIG{__WARN__} = sub { usageerr($_[0]) };
    GetOptions(\%options, @options_spec);
}

if (not (@ARGV >= 1 and @ARGV <= 3)) {
    usageerr(_g('one to three arguments expected'));
}

my $type = defined($options{type}) ? $options{type} : 'deb';
my $arch = $options{arch};

my @find_args;
if ($options{arch}) {
     @find_args = ('(', '-name', "*_all.$type", '-o',
			'-name', "*_${arch}.$type", ')');
}
else {
     @find_args = ('-name', "*.$type");
}

my ($binarydir, $override, $pathprefix) = @ARGV;

if (not -d $binarydir) {
    error(_g('binary dir %s not found'), $binarydir);
}
if (defined $override and not -e $override) {
    error(_g('override file %s not found'), $override);
}

$pathprefix //= '';

my $find_h = IO::Handle->new();
open($find_h, '-|', 'find', '-L', "$binarydir/", @find_args, '-print')
     or syserr(_g("couldn't open %s for reading"), $binarydir);
FILE:
    while (<$find_h>) {
	chomp;
	my $fn = $_;
	my $output;
	my $pid = spawn(exec => [ 'dpkg-deb', '-I', $fn, 'control' ],
	                to_pipe => \$output);
	my $fields = Dpkg::Control->new(type => CTRL_INDEX_PKG);
	$fields->parse($output, $fn)
	    or error(_g("couldn't parse control information from %s"), $fn);
	wait_child($pid, no_check => 1);
	if ($?) {
	    warning(_g("\`dpkg-deb -I %s control' exited with %d, skipping package"),
	            $fn, $?);
	    next;
	}

	defined($fields->{'Package'})
	    or error(_g('no Package field in control file of %s'), $fn);
	my $p = $fields->{'Package'};

	if (defined($packages{$p}) and not $options{multiversion}) {
	    foreach (@{$packages{$p}}) {
		if (version_compare_relation($fields->{'Version'}, REL_GT,
					     $_->{'Version'}))
                {
		    warning(_g('package %s (filename %s) is repeat but newer version;'),
		            $p, $fn);
		    warning(_g('used that one and ignored data from %s!'),
		            $_->{Filename});
		    $packages{$p} = [];
		} else {
		    warning(_g('package %s (filename %s) is repeat;'), $p, $fn);
		    warning(_g('ignored that one and using data from %s!'),
		            $_->{Filename});
		    next FILE;
		}
	    }
	}
	warning(_g('package %s (filename %s) has Filename field!'), $p, $fn)
	    if defined($fields->{'Filename'});

	$fields->{'Filename'} = "$pathprefix$fn";

        my $sums = Dpkg::Checksums->new();
	$sums->add_from_file($fn);
        foreach my $alg (checksums_get_list()) {
            if ($alg eq 'md5') {
	        $fields->{'MD5sum'} = $sums->get_checksum($fn, $alg);
            } else {
                $fields->{$alg} = $sums->get_checksum($fn, $alg);
            }
        }
	$fields->{'Size'} = $sums->get_size($fn);
        $fields->{'X-Medium'} = $options{medium} if defined $options{medium};

	push @{$packages{$p}}, $fields;
    }
close($find_h);

load_override($override) if defined $override;
load_override_extra($options{'extra-override'}) if defined $options{'extra-override'};

my @missingover=();

my $records_written = 0;
for my $p (sort keys %packages) {
    if (defined($override) and not defined($overridden{$p})) {
        push(@missingover,$p);
    }
    for my $package (@{$packages{$p}}) {
         print("$package\n") or syserr(_g('failed when writing stdout'));
         $records_written++;
    }
}
close(STDOUT) or syserr(_g("couldn't close stdout"));

if (@changedmaint) {
    warning(_g('Packages in override file with incorrect old maintainer value:'));
    warning($_) foreach (@changedmaint);
}
if (@samemaint) {
    warning(_g('Packages specifying same maintainer as override file:'));
    warning($_) foreach (@samemaint);
}
if (@missingover) {
    warning(_g('Packages in archive but missing from override file:'));
    warning('  %s', join(' ', @missingover));
}
if (@spuriousover) {
    warning(_g('Packages in override file but not in archive:'));
    warning('  %s', join(' ', @spuriousover));
}

info(_g('Wrote %s entries to output Packages file.'), $records_written);
