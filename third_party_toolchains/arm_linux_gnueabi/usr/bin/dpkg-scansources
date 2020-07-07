#!/usr/bin/perl
#
# Copyright © 1999 Roderick Schertler
# Copyright © 2002 Wichert Akkerman <wakkerma@debian.org>
# Copyright © 2006-2009,2011-2012 Guillem Jover <guillem@debian.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or (at
# your option) any later version.
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
use Dpkg::Util qw(:list);
use Dpkg::Control;
use Dpkg::Checksums;
use Dpkg::Compression::FileHandle;
use Dpkg::Compression;

textdomain('dpkg-dev');

# Errors with a single package are warned about but don't affect the
# exit code. Only errors which affect everything cause a non-zero exit.
my $exit = 0;

# %override is a hash of lists.  The subs following describe what's in
# the lists.

my %override;
sub O_PRIORITY          () { 0 }
sub O_SECTION           () { 1 }
sub O_MAINT_FROM        () { 2 } # undef for non-specific, else listref
sub O_MAINT_TO          () { 3 } # undef if there's no maint override
my %extra_override;

my %priority = (
    'extra' => 1,
    'optional' => 2,
    'standard' => 3,
    'important' => 4,
    'required' => 5,
);

# Switches

my $debug = 0;
my $no_sort = 0;
my $src_override = undef;
my $extra_override_file = undef;

my @option_spec = (
    'debug!' => \$debug,
    'help|?' => sub { usage(); exit 0; },
    'no-sort|n' => \$no_sort,
    'source-override|s=s' => \$src_override,
    'extra-override|e=s' => \$extra_override_file,
    'version' => \&version,
);

sub debug {
    print @_, "\n" if $debug;
}

sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;
    exit;
}

sub usage {
    printf _g(
"Usage: %s [<option>...] <binary-path> [<override-file> [<path-prefix>]] > Sources

Options:
  -n, --no-sort            don't sort by package before outputting.
  -e, --extra-override <file>
                           use extra override file.
  -s, --source-override <file>
                           use file for additional source overrides, default
                           is regular override file with .src appended.
      --debug              turn debugging on.
  -?, --help               show this help message.
      --version            show the version.

See the man page for the full documentation.
"), $Dpkg::PROGNAME;
}

sub close_msg {
    my $name = shift;
    return sprintf(_g("error closing %s (\$? %d, \$! `%s')"),
                   $name, $?, $!)."\n";
}

sub load_override {
    my $file = shift;
    local $_;

    my $comp_file = Dpkg::Compression::FileHandle->new(filename => $file);
    while (<$comp_file>) {
    	s/#.*//;
	next if /^\s*$/;
	s/\s+$//;

	my @data = split ' ', $_, 4;
	unless (@data == 3 || @data == 4) {
	    warning(_g('invalid override entry at line %d (%d fields)'),
	            $., 0 + @data);
	    next;
	}
	my ($package, $priority, $section, $maintainer) = @data;
	if (exists $override{$package}) {
	    warning(_g('ignoring duplicate override entry for %s at line %d'),
	            $package, $.);
	    next;
	}
	if (!$priority{$priority}) {
	    warning(_g('ignoring override entry for %s, invalid priority %s'),
	            $package, $priority);
	    next;
	}

	$override{$package} = [];
	$override{$package}[O_PRIORITY] = $priority;
	$override{$package}[O_SECTION] = $section;
	if (!defined $maintainer) {
	    # do nothing
	}
	elsif ($maintainer =~ /^(.*\S)\s*=>\s*(.*)$/) {
	    $override{$package}[O_MAINT_FROM] = [split m{\s*//\s*}, $1];
	    $override{$package}[O_MAINT_TO] = $2;
	}
	else {
	    $override{$package}[O_MAINT_TO] = $maintainer;
	}
    }
    close($comp_file);
}

sub load_src_override {
    my ($user_file, $regular_file) = @_;
    my ($file);
    local $_;

    if (defined $user_file) {
	$file = $user_file;
    }
    elsif (defined $regular_file) {
        my $comp = compression_guess_from_filename($regular_file);
        if (defined($comp)) {
	    $file = $regular_file;
	    my $ext = compression_get_property($comp, 'file_ext');
            $file =~ s/\.$ext$/.src.$ext/;
        } else {
	    $file = "$regular_file.src";
        }
        return unless -e $file;
    }
    else {
	return;
    }

    debug "source override file $file";
    my $comp_file = Dpkg::Compression::FileHandle->new(filename => $file);
    while (<$comp_file>) {
    	s/#.*//;
	next if /^\s*$/;
	s/\s+$//;

	my @data = split ' ', $_;
	unless (@data == 2) {
	    warning(_g('invalid source override entry at line %d (%d fields)'),
	            $., 0 + @data);
	    next;
	}

	my ($package, $section) = @data;
	my $key = "source/$package";
	if (exists $override{$key}) {
	    warning(_g('ignoring duplicate source override entry for %s at line %d'),
	            $package, $.);
	    next;
	}
	$override{$key} = [];
	$override{$key}[O_SECTION] = $section;
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
        $extra_override{$p}{$field} = $value;
    }
    close($comp_file);
}

# Given PREFIX and DSC-FILE, process the file and returns the fields.

sub process_dsc {
    my ($prefix, $file) = @_;

    my $basename = $file;
    my $dir = ($basename =~ s{^(.*)/}{}) ? $1 : '';
    $dir = "$prefix$dir";
    $dir =~ s{/+$}{};
    $dir = '.' if $dir eq '';

    # Parse ‘.dsc’ file.
    my $fields = Dpkg::Control->new(type => CTRL_PKG_SRC);
    $fields->load($file);
    $fields->set_options(type => CTRL_INDEX_SRC);

    # Get checksums
    my $checksums = Dpkg::Checksums->new();
    $checksums->add_from_file($file, key => $basename);
    $checksums->add_from_control($fields, use_files_for_md5 => 1);

    my $source = $fields->{Source};
    my @binary = split /\s*,\s*/, $fields->{Binary};

    error(_g('no binary packages specified in %s'), $file) unless (@binary);

    # Rename the source field to package.
    $fields->{Package} = $fields->{Source};
    delete $fields->{Source};

    # The priority for the source package is the highest priority of the
    # binary packages it produces.
    my @binary_by_priority = sort {
	    ($override{$a} ? $priority{$override{$a}[O_PRIORITY]} : 0)
		<=>
	    ($override{$b} ? $priority{$override{$b}[O_PRIORITY]} : 0)
	} @binary;
    my $priority_override = $override{$binary_by_priority[-1]};
    my $priority = $priority_override
			? $priority_override->[O_PRIORITY]
			: undef;
    $fields->{Priority} = $priority if defined $priority;

    # For the section override, first check for a record from the source
    # override file, else use the regular override file.
    my $section_override = $override{"source/$source"} || $override{$source};
    my $section = $section_override
			? $section_override->[O_SECTION]
			: undef;
    $fields->{Section} = $section if defined $section;

    # For the maintainer override, use the override record for the first
    # binary. Modify the maintainer if necessary.
    my $maintainer_override = $override{$binary[0]};
    if ($maintainer_override && defined $maintainer_override->[O_MAINT_TO]) {
        if (!defined $maintainer_override->[O_MAINT_FROM] ||
            any { $fields->{Maintainer} eq $_ }
                @{ $maintainer_override->[O_MAINT_FROM] }) {
            $fields->{Maintainer} = $maintainer_override->[O_MAINT_TO];
        }
    }

    # Process extra override
    if (exists $extra_override{$source}) {
        my ($field, $value);
        while(($field, $value) = each %{$extra_override{$source}}) {
            $fields->{$field} = $value;
        }
    }

    # A directory field will be inserted just before the files field.
    $fields->{Directory} = $dir;

    $checksums->export_to_control($fields, use_files_for_md5 => 1);

    return $fields;
}

sub main {
    my (@out);

    {
        local $SIG{__WARN__} = sub { usageerr($_[0]) };
        GetOptions(@option_spec);
    }
    usageerr(_g('one to three arguments expected'))
        if @ARGV < 1 or @ARGV > 3;

    push @ARGV, undef if @ARGV < 2;
    push @ARGV, '' if @ARGV < 3;
    my ($dir, $override, $prefix) = @ARGV;

    load_override $override if defined $override;
    load_src_override $src_override, $override;
    load_override_extra $extra_override_file if defined $extra_override_file;

    open my $find_fh, '-|', "find -L \Q$dir\E -name '*.dsc' -print"
        or syserr(_g('cannot fork for %s'), 'find');
    while (<$find_fh>) {
    	chomp;
	s{^\./+}{};

        my $fields;

        # FIXME: Fix it instead to not die on syntax and general errors?
        eval {
            $fields = process_dsc($prefix, $_);
        };
        if ($@) {
            warn $@;
            next;
        }

	if ($no_sort) {
            $fields->output(\*STDOUT);
            print "\n";
	}
	else {
            push @out, $fields;
	}
    }
    close $find_fh or error(close_msg, 'find');

    if (@out) {
        foreach my $dsc (sort { $a->{Package} cmp $b->{Package} } @out) {
            $dsc->output(\*STDOUT);
            print "\n";
        }
    }

    return 0;
}

$exit = main || $exit;
$exit = 1 if $exit and not $exit % 256;
exit $exit;
