#!/usr/bin/perl

# Copyright © 2009-2010 Raphaël Hertzog <hertzog@debian.org>
# Copyright © 2012 Guillem Jover <guillem@debian.org>
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

use Dpkg ();
use Dpkg::Changelog::Debian;
use Dpkg::ErrorHandling;
use Dpkg::Gettext;
use Dpkg::Version;

use Getopt::Long qw(:config posix_default bundling no_ignorecase);
use Scalar::Util qw(blessed);

textdomain('dpkg-dev');

sub merge_entries($$$);
sub merge_block($$$;&);
sub merge_entry_item($$$$);
sub merge_conflict($$);
sub get_conflict_block($$);
sub join_lines($);

BEGIN {
    eval 'use Algorithm::Merge qw(merge);';
    if ($@) {
        eval q{
            sub merge {
                my ($o, $a, $b) = @_;
                return @$a if join("\n", @$a) eq join("\n", @$b);
                return get_conflict_block($a, $b);
            }
        };
    }
}

sub version {
    printf _g("Debian %s version %s.\n"), $Dpkg::PROGNAME, $Dpkg::PROGVERSION;

    printf "\n" . _g(
'This is free software; see the GNU General Public License version 2 or
later for copying conditions. There is NO warranty.
');
}

sub usage {
    printf(_g(
"Usage: %s [<option>...] <old> <new-a> <new-b> [<out>]

Options:
  -m, --merge-prereleases  merge pre-releases together, ignores everything
                           after the last '~' in the version.
  -?, --help               show this help message.
      --version            show the version.
"), $Dpkg::PROGNAME);
}

my $merge_prereleases;

my @options_spec = (
    'help|?' => sub { usage(); exit(0) },
    'version' => sub { version(); exit(0) },
    'merge-prereleases|m' => \$merge_prereleases,
);

{
    local $SIG{__WARN__} = sub { usageerr($_[0]) };
    GetOptions(@options_spec);
}

my ($old, $new_a, $new_b, $out_file) = @ARGV;
unless (defined $old and defined $new_a and defined $new_b)
{
    usageerr(_g('needs at least three arguments'));
}
unless (-e $old and -e $new_a and -e $new_b)
{
    usageerr(_g('file arguments need to exist'));
}

my ($cho, $cha, $chb);
$cho = Dpkg::Changelog::Debian->new();
$cho->load($old);
$cha = Dpkg::Changelog::Debian->new();
$cha->load($new_a);
$chb = Dpkg::Changelog::Debian->new();
$chb->load($new_b);

my @o = reverse @$cho;
my @a = reverse @$cha;
my @b = reverse @$chb;

my @result; # Lines to output
my $exitcode = 0; # 1 if conflict encountered

unless (merge_block($cho, $cha, $chb, sub {
			my $tail = $_[0]->get_unparsed_tail();
			chomp $tail if defined $tail;
			return $tail;
		    }))
{
    merge_conflict($cha->get_unparsed_tail(), $chb->get_unparsed_tail());
}

while (1) {
    my ($o, $a, $b) = get_items_to_merge();
    last unless defined $o or defined $a or defined $b;
    next if merge_block($o, $a, $b);
    # We only have the usually conflicting cases left
    if (defined $a and defined $b) {
	# Same entry, merge sub-items separately for a nicer result
	merge_entries($o, $a, $b);
    } else {
	# Non-existing on one side, changed on the other side
	merge_conflict($a, $b);
    }
}

if (defined($out_file) and $out_file ne '-') {
    open(my $out_fh, '>', $out_file)
        or syserr(_g('cannot write %s'), $out_file);
    print { $out_fh } ((blessed $_) ? "$_" : "$_\n") foreach @result;
    close($out_fh) or syserr(_g('cannot write %s'), $out_file);
} else {
    print ((blessed $_) ? "$_" : "$_\n") foreach @result;
}

exit $exitcode;

# Returns the next items to merge, all items returned correspond to the
# same minimal version among the 3 possible next items (undef is returned
# if the next item on the given changelog is skipped)
sub get_items_to_merge {
    my @items = (shift @o, shift @a, shift @b);
    my @arrays = (\@o, \@a, \@b);
    my $minver;
    foreach my $i (0 .. 2) {
	if (defined $minver and defined $items[$i]) {
	    my $cmp = compare_versions($minver, $items[$i]->get_version());
	    if ($cmp > 0) {
		$minver = $items[$i]->get_version();
		foreach my $j (0 .. $i - 1) {
		    unshift @{$arrays[$j]}, $items[$j];
		    $items[$j] = undef;
		}
	    } elsif ($cmp < 0) {
		unshift @{$arrays[$i]}, $items[$i];
		$items[$i] = undef;
	    }
	} else {
	    $minver = $items[$i]->get_version() if defined $items[$i];
	}
    }
    return @items;
}

# Compares the versions taking into account some oddities like the fact
# that we want backport/volatile versions to sort higher than the version
# on which they are based.
sub compare_versions {
    my ($a, $b) = @_;
    return 0 if not defined $a and not defined $b;
    return 1 if not defined $b;
    return -1 if not defined $a;
    $a = $a->get_version() if ref($a) and $a->isa('Dpkg::Changelog::Entry');
    $b = $b->get_version() if ref($b) and $b->isa('Dpkg::Changelog::Entry');
    # Backport and volatile are not real prereleases
    $a =~ s/~(bpo|vola)/+$1/;
    $b =~ s/~(bpo|vola)/+$1/;
    if ($merge_prereleases) {
	$a =~ s/~[^~]*$//;
	$b =~ s/~[^~]*$//;
    }
    $a = Dpkg::Version->new($a);
    $b = Dpkg::Version->new($b);
    return $a <=> $b;
}

# Merge changelog entries smartly by merging individually the different
# parts constituting an entry
sub merge_entries($$$) {
    my ($o, $a, $b) = @_;
    # NOTE: Only $o can be undef

    # Merge the trailer line
    unless (merge_entry_item('blank_after_trailer', $o, $a, $b)) {
	unshift @result, '';
    }
    unless (merge_entry_item('trailer', $o, $a, $b)) {
	merge_conflict($a->get_part('trailer'), $b->get_part('trailer'));
    }

    # Merge the changes
    unless (merge_entry_item('blank_after_changes', $o, $a, $b)) {
	unshift @result, '';
    }
    my @merged = merge(defined $o ? $o->get_part('changes') : [],
		       $a->get_part('changes'), $b->get_part('changes'),
		       {
			   CONFLICT => sub {
				$exitcode = 1;
				return get_conflict_block($_[0], $_[1]);
			   }
		       });
    unshift @result, @merged;

    # Merge the header line
    unless (merge_entry_item('blank_after_header', $o, $a, $b)) {
	unshift @result, '';
    }
    unless (merge_entry_item('header', $o, $a, $b)) {
	merge_conflict($a->get_part('header'), $b->get_part('header'));
    }
}

sub join_lines($) {
    my $array = shift;
    return join("\n", @$array) if ref($array) eq 'ARRAY';
    return $array;
}

# Try to merge the obvious cases, return 1 on success and 0 on failure
# O A B
# - x x => x
# o o b => b
# - - b => b
# o a o => a
# - a - => a
sub merge_block($$$;&) {
    my ($o, $a, $b, $preprocess) = @_;
    $preprocess //= \&join_lines;
    $o = &$preprocess($o) if defined($o);
    $a = &$preprocess($a) if defined($a);
    $b = &$preprocess($b) if defined($b);
    return 1 if not defined($a) and not defined($b);
    if (defined($a) and defined($b) and ($a eq $b)) {
	unshift @result, $a;
    } elsif ((defined($a) and defined($o) and ($a eq $o)) or
	     (not defined($a) and not defined($o))) {
	unshift @result, $b if defined $b;
    } elsif ((defined($b) and defined($o) and ($b eq $o)) or
	     (not defined($b) and not defined($o))) {
	unshift @result, $a if defined $a;
    } else {
	return 0;
    }
    return 1;
}

sub merge_entry_item($$$$) {
    my ($item, $o, $a, $b) = @_;
    if (blessed($o) and $o->isa('Dpkg::Changelog::Entry')) {
	$o = $o->get_part($item);
    } elsif (ref $o) {
	$o = $o->{$item};
    }
    if (blessed($a) and $a->isa('Dpkg::Changelog::Entry')) {
	$a = $a->get_part($item);
    } elsif (ref $a) {
	$a = $a->{$item};
    }
    if (blessed($b) and $b->isa('Dpkg::Changelog::Entry')) {
	$b = $b->get_part($item);
    } elsif (ref $b) {
	$b = $b->{$item};
    }
    return merge_block($o, $a, $b);
}

sub merge_conflict($$) {
    my ($a, $b) = @_;
    unshift @result, get_conflict_block($a, $b);
    $exitcode = 1;
}

sub get_conflict_block($$) {
    my ($a, $b) = @_;
    my (@a, @b);
    push @a, $a if defined $a;
    push @b, $b if defined $b;
    @a = @{$a} if ref($a) eq 'ARRAY';
    @b = @{$b} if ref($b) eq 'ARRAY';
    return ('<<<<<<<', @a, '=======', @b, '>>>>>>>');
}
