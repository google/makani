#!/usr/bin/perl
#
# dpkg-name
#
# Copyright © 1995,1996 Erick Branderhorst <branderh@debian.org>.
# Copyright © 2006-2010,2012-2013 Guillem Jover <guillem@debian.org>
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

use File::Basename;
use File::Path;

use Dpkg ();
use Dpkg::Gettext;
use Dpkg::ErrorHandling;
use Dpkg::Control;
use Dpkg::Arch qw(get_host_arch);

textdomain('dpkg-dev');

my %options = (
    subdir => 0,
    destdir => '',
    createdir => 0,
    overwrite => 0,
    symlink => 0,
    architecture => 1,
);

sub version()
{
    printf(_g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION);
}

sub usage()
{
    printf(_g("Usage: %s [<option>...] <file>...\n"), $Dpkg::PROGNAME);

    print(_g("
Options:
  -a, --no-architecture    no architecture part in filename.
  -o, --overwrite          overwrite if file exists.
  -k, --symlink            don't create a new file, but a symlink.
  -s, --subdir [dir]       move file into subdir (use with care).
  -c, --create-dir         create target dir if not there (use with care).
  -?, --help               show this help message.
  -v, --version            show the version.

file.deb changes to <package>_<version>_<architecture>.<package_type>
according to the 'underscores convention'.
"));
}

sub fileexists($)
{
    my ($filename) = @_;

    if (-f $filename) {
        return 1;
    } else {
        warning(_g("cannot find '%s'"), $filename);
        return 0;
    }
}

sub filesame($$)
{
    my ($a, $b) = @_;
    my @sta = stat($a);
    my @stb = stat($b);

    # Same device and inode numbers.
    return (@sta and @stb and $sta[0] == $stb[0] and $sta[1] == $stb[1]);
}

sub getfields($)
{
    my ($filename) = @_;

    # Read the fields
    open(my $cdata_fh, '-|', 'dpkg-deb', '-f', '--', $filename)
        or syserr(_g('cannot open %s'), $filename);
    my $fields = Dpkg::Control->new(type => CTRL_PKG_DEB);
    $fields->parse($cdata_fh, sprintf(_g('binary control file %s'), $filename));
    close($cdata_fh);

    return $fields;
}

sub getarch($$)
{
    my ($filename, $fields) = @_;

    my $arch = $fields->{Architecture};
    if (not $fields->{Architecture} and $options{architecture}) {
        $arch = get_host_arch();
        warning(_g("assuming architecture '%s' for '%s'"), $arch, $filename);
    }

    return $arch;
}

sub getname($$$)
{
    my ($filename, $fields, $arch) = @_;

    my $pkg = $fields->{Package};
    (my $version = $fields->{Version}) =~ s/.*://;
    my $type = $fields->{'Package-Type'} || 'deb';

    my $tname;
    if ($options{architecture}) {
        $tname = "$pkg\_$version\_$arch.$type";
    } else {
        $tname = "$pkg\_$version.$type";
    }
    (my $name = $tname) =~ s/ //g;
    if ($tname ne $name) { # control fields have spaces
        warning(_g("bad package control information for '%s'"), $filename);
    }
    return $name;
}

sub getdir($$$)
{
    my ($filename, $fields, $arch) = @_;
    my $dir;

    if (!$options{destdir}) {
        $dir = dirname($filename);
        if ($options{subdir}) {
            my $section = $fields->{Section};
            if (!$section) {
                $section = 'no-section';
                warning(_g("assuming section '%s' for '%s'"), $section,
                        $filename);
            }
            if ($section ne 'non-free' and $section ne 'contrib' and
                $section ne 'no-section') {
                $dir = "unstable/binary-$arch/$section";
            } else {
                $dir = "$section/binary-$arch";
            }
        }
    } else {
        $dir = $options{destdir};
    }

    return $dir;
}

sub move($)
{
    my ($filename) = @_;

    if (fileexists($filename)) {
        my $fields = getfields($filename);

        unless (exists $fields->{Package}) {
            warning(_g("no Package field found in '%s', skipping it"),
                    $filename);
            return;
        }

        my $arch = getarch($filename, $fields);

        my $name = getname($filename, $fields, $arch);

        my $dir = getdir($filename, $fields, $arch);
        if (! -d $dir) {
            if ($options{createdir}) {
                if (mkpath($dir)) {
                    info(_g("created directory '%s'"), $dir);
                } else {
                    error(_g("cannot create directory '%s'"), $dir);
                }
            } else {
                error(_g("no such directory '%s', try --create-dir (-c) option"),
                      $dir);
            }
        }

        my $newname = "$dir/$name";

        my @command;
        if ($options{symlink}) {
            @command = qw(ln -s --);
        } else {
            @command = qw(mv --);
        }

        if (filesame($newname, $filename)) {
            warning(_g("skipping '%s'"), $filename);
        } elsif (-f $newname and not $options{overwrite}) {
            warning(_g("cannot move '%s' to existing file"), $filename);
        } elsif (system(@command, $filename, $newname) == 0) {
            info(_g("moved '%s' to '%s'"), basename($filename), $newname);
        } else {
            error(_g('mkdir can be used to create directory'));
        }
    }
}

my @files;

while (@ARGV) {
    $_ = shift(@ARGV);
    if (m/^-\?|--help$/) {
        usage();
        exit(0);
    } elsif (m/^-v|--version$/) {
        version();
        exit(0);
    } elsif (m/^-c|--create-dir$/) {
        $options{createdir} = 1;
    } elsif (m/^-s|--subdir$/) {
        $options{subdir} = 1;
        if (-d $ARGV[0]) {
            $options{destdir} = shift(@ARGV);
        }
    } elsif (m/^-o|--overwrite$/) {
        $options{overwrite} = 1;
    } elsif (m/^-k|--symlink$/) {
        $options{symlink} = 1;
    } elsif (m/^-a|--no-architecture$/) {
        $options{architecture} = 0;
    } elsif (m/^--$/) {
        push @files, @ARGV;
        last;
    } elsif (m/^-/) {
        usageerr(_g("unknown option \`%s'"), $_);
    } else {
        push @files, $_;
    }
}

@files or usageerr(_g('need at least a filename'));

foreach my $file (@files) {
    move($file);
}

0;
