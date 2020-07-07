#!/usr/bin/perl
#
# dpkg-distaddfile
#
# Copyright © 1996 Ian Jackson
# Copyright © 2006-2008,2010,2012-2013 Guillem Jover <guillem@debian.org>
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
use Dpkg::File;

textdomain('dpkg-dev');

my $fileslistfile = 'debian/files';


sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    printf _g('
This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf _g(
'Usage: %s [<option>...] <filename> <section> <priority>

Options:
  -f<files-list-file>      write files here instead of debian/files.
  -?, --help               show this help message.
      --version            show the version.
'), $Dpkg::PROGNAME;
}

while (@ARGV && $ARGV[0] =~ m/^-/) {
    $_=shift(@ARGV);
    if (m/^-f/) {
        $fileslistfile= $';
    } elsif (m/^-(\?|-help)$/) {
        usage();
        exit(0);
    } elsif (m/^--version$/) {
        version();
        exit(0);
    } elsif (m/^--$/) {
        last;
    } else {
        usageerr(_g("unknown option \`%s'"), $_);
    }
}
usageerr(_g('need exactly a filename, section and priority')) if @ARGV != 3;

my ($file, $section, $priority) = @ARGV;

($file =~ m/\s/ || $section =~ m/\s/ || $priority =~ m/\s/) &&
    error(_g('filename, section and priority may contain no whitespace'));

# Obtain a lock on debian/control to avoid simultaneous updates
# of debian/files when parallel building is in use
my $lockfh;
my $lockfile = 'debian/control';
sysopen($lockfh, $lockfile, O_WRONLY)
    or syserr(_g('cannot write %s'), $lockfile);
file_lock($lockfh, $lockfile);

open(my $fileslistnew_fh, '>', "$fileslistfile.new")
    or syserr(_g('open new files list file'));
if (open(my $fileslist_fh, '<', $fileslistfile)) {
    while (<$fileslist_fh>) {
        s/\n$//;
        next if m/^(\S+) / && $1 eq $file;
        print { $fileslistnew_fh } "$_\n"
            or syserr(_g('copy old entry to new files list file'));
    }
    close $fileslist_fh or syserr(_g('cannot close %s'), $fileslistfile);
} elsif ($! != ENOENT) {
    syserr(_g('read old files list file'));
}
print { $fileslistnew_fh } "$file $section $priority\n"
    or syserr(_g('write new entry to new files list file'));
close($fileslistnew_fh) or syserr(_g('close new files list file'));
rename("$fileslistfile.new", $fileslistfile)
    or syserr(_g('install new files list file'));

# Release the lock
close($lockfh) or syserr(_g('cannot close %s'), $lockfile);
