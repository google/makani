#!/usr/bin/perl
#
# dpkg-parsechangelog
#
# Copyright © 1996 Ian Jackson
# Copyright © 2001 Wichert Akkerman
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
use Dpkg::Changelog::Parse;

textdomain('dpkg-dev');

my %options;
my $fieldname;

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
  -l<changelog-file>       get per-version info from this file.
  -F<changelog-format>     force changelog format.
  -L<libdir>               look for changelog parsers in <libdir>.
  -S, --show-field <field> show the values for <field>.
  -?, --help               show this help message.
      --version            show the version.')
    . "\n\n" . _g(
"Parser options:
    --format <output-format>    see man page for list of available
                                output formats, defaults to 'dpkg'
                                for compatibility with dpkg-dev
    --since <version>,          include all changes later than version
      -s<version>, -v<version>
    --until <version>,          include all changes earlier than version
      -u<version>
    --from <version>,           include all changes equal or later
      -f<version>               than version
    --to <version>, -t<version> include all changes up to or equal
                                than version
    --count <number>,           include <number> entries from the top
      -c<number>, -n<number>    (or the tail if <number> is lower than 0)
    --offset <number>,          change the starting point for --count,
      -o<number>                counted from the top (or the tail if
                                <number> is lower than 0)
    --all                       include all changes
"), $Dpkg::PROGNAME;
}

while (@ARGV) {
    last unless $ARGV[0] =~ m/^-/;
    $_ = shift(@ARGV);
    if (m/^-L(.+)$/) {
	$options{libdir} = $1;
    } elsif (m/^-F([0-9a-z]+)$/) {
	$options{changelogformat} = $1;
    } elsif (m/^-l(.+)$/) {
	$options{file} = $1;
    } elsif (m/^-S(.+)$/) {
	$fieldname = $1;
    } elsif (m/^--show-field(?:=(.+))?$/) {
	$fieldname = $1 // shift(@ARGV);
    } elsif (m/^--$/) {
	last;
    } elsif (m/^-([cfnostuv])(.*)$/) {
	if (($1 eq 'c') or ($1 eq 'n')) {
	    $options{count} = $2;
	} elsif ($1 eq 'f') {
	    $options{from} = $2;
	} elsif ($1 eq 'o') {
	    $options{offset} = $2;
	} elsif (($1 eq 's') or ($1 eq 'v')) {
	    $options{since} = $2;
	} elsif ($1 eq 't') {
	    $options{to} = $2;
	} elsif ($1 eq 'u') {
	    ## no critic (ControlStructures::ProhibitUntilBlocks)
	    $options{until} = $2;
	    ## use critic
	}
    } elsif (m/^--(count|file|format|from|offset|since|to|until)(.*)$/) {
	if ($2) {
	    $options{$1} = $2;
	} else {
	    $options{$1} = shift(@ARGV);
	}
    } elsif (m/^--all$/) {
	$options{all} = undef;
    } elsif (m/^-(\?|-help)$/) {
	usage(); exit(0);
    } elsif (m/^--version$/) {
	version(); exit(0);
    } else {
	usageerr(_g("unknown option \`%s'"), $_);
    }
}
usageerr(_g('takes no non-option arguments')) if @ARGV;

my $count = 0;
my @fields = changelog_parse(%options);
foreach my $f (@fields) {
    print "\n" if $count++;
    if ($fieldname) {
        print $f->{$fieldname} . "\n";
    } else {
        print $f->output();
    }
}
