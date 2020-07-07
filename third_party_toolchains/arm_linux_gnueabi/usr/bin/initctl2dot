#!/usr/bin/python3
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#
# Copyright © 2011 Canonical Ltd.
#
# Author: James Hunt <james.hunt@canonical.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2, as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#---------------------------------------------------------------------

#---------------------------------------------------------------------
# Script to take output of "initctl show-config -e" and convert it into
# a Graphviz DOT language (".dot") file for procesing with dot(1), etc.
#
# Notes:
#
# - Slightly laborious logic used to satisfy graphviz requirement that
#   all nodes be defined before being referenced.
#
# Usage:
#
#   initctl show-config -e > initctl.out
#   initctl2dot -f initctl.out -o upstart.dot
#   dot -Tpng -o upstart.png upstart.dot
#
# Or more simply:
#
#  initctl2dot -o - | dot -Tpng -o upstart.png
#
# See also:
#
# - dot(1).
# - initctl(8).
# - http://www.graphviz.org.
#---------------------------------------------------------------------

import sys
import re
import fnmatch
import os
import datetime
from subprocess import Popen, PIPE
from argparse import ArgumentParser

options = None
jobs = {}
events = {}
script_name = os.path.basename(sys.argv[0])

cmd = None
use_system = True
upstart_session = None

# list of jobs to restict output to
restrictions_list = []

sanitise_table = str.maketrans({
    '-': '_',
    '$': 'dollar_',
    '[': 'lbracker',
    ']': 'rbracker',
    '!': 'bang',
    ':': 'colon',
    '*': 'star',
    '?': 'question',
    '.': 'dot',
    '/': 'slash',
})

default_color_emits = 'green'
default_color_start_on = 'blue'
default_color_stop_on = 'red'
default_color_event = 'thistle'
default_color_job = '#DCDCDC'  # "Gainsboro"
default_color_text = 'black'
default_color_bg = 'white'

default_outfile = 'upstart.dot'


def header(ofh):
    ofh.write("""digraph upstart {{
  node [shape=\"diamond\", fontcolor=\"{options.color_event_text}\", """
              """fillcolor=\"{options.color_event}\", style=\"filled\"];
  rankdir=LR;
  overlap=false;
  bgcolor=\"{options.color_bg}\";
  fontcolor=\"{options.color_text}\";
""".format(options=options))


def footer(ofh):
    global upstart_session
    global use_system

    details = ''

    if use_system:
        details += "\\nfor the system\\n"
    else:
        details += "\\nfor session '%s'\\n" % upstart_session

    if options.restrictions:
        details += "(subset, "
    else:
        details += "("

    if options.infile:
        details += "from file data)."
    else:
        details += "from '%s' on host %s)." % (cmd, os.uname()[1])

    ofh.write("  overlap=false;\n"
              "  label=\"Generated on {datenow} by {script_name} {details}\\n"
              "Boxes of color {options.color_job} denote jobs.\\n"
              "Solid diamonds of color {options.color_event} denote events.\\n"
              "Dotted diamons denote 'glob' events.\\n"
              "Emits denoted by {options.color_emits} lines.\\n"
              "Start on denoted by {options.color_start_on} lines.\\n"
              "Stop on denoted by {options.color_stop_on} lines.\\n"
              "\";\n"
              "}}\n".format(options=options, datenow=datetime.datetime.now(),
                            script_name=script_name, details=details))


# Map punctuation to symbols palatable to graphviz
# (which amongst other things dislikes dashes in node names)
def sanitise(s):
    global sanitise_table

    return s.translate(sanitise_table)


# Convert a dollar in @name to a unique-ish new name, based on @job and
# return it. Used for very rudimentary instance handling.
def encode_dollar(job, name):
    if name[0] == '$':
        name = job + ':' + name
    return name


# Jobs and events can have identical names, so prefix them to namespace
# them off.
def mk_job_node_name(name):
    return sanitise('job_' + name)


def mk_event_node_name(name):
    return sanitise('event_' + name)


def show_event(ofh, name):
    str = "  %s [label=\"%s\", shape=diamond, fontcolor=\"%s\", " \
          "fillcolor=\"%s\"," % (mk_event_node_name(name), name,
                                 options.color_event_text, options.color_event)

    if '*' in name:
        str += " style=\"dotted\""
    else:
        str += " style=\"filled\""

    str += "];\n"

    ofh.write(str)


def show_events(ofh):
    events_to_show = []

    if restrictions_list:
        for job in restrictions_list:

            # We want all events emitted by the jobs in the restrictions_list.
            events_to_show += jobs[job]['emits']

            # We also want all events that jobs in restrictions_list start/stop
            # on.
            events_to_show += jobs[job]['start on']['event']
            events_to_show += jobs[job]['stop on']['event']

            # We also want all events emitted by all jobs that jobs in the
            # restrictions_list start/stop on. Finally, we want all events
            # emmitted by those jobs in the restrictions_list that we
            # start/stop on.
            for j in jobs[job]['start on']['job']:
                if j in jobs and 'emits' in jobs[j]:
                    events_to_show += jobs[j]['emits']

            for j in jobs[job]['stop on']['job']:
                if j in jobs and 'emits' in jobs[j]:
                    events_to_show += jobs[j]['emits']
    else:
        events_to_show = events

    for e in events_to_show:
        show_event(ofh, e)


def show_job(ofh, name):
    ofh.write("  %s [shape=\"record\", label=\"<job> %s | { <start> start on |"
              " <stop> stop on }\", fontcolor=\"%s\", style=\"filled\", "
              " fillcolor=\"%s\"];\n" % (mk_job_node_name(name), name,
                                         options.color_job_text,
                                         options.color_job))


def show_jobs(ofh):
    if restrictions_list:
        jobs_to_show = restrictions_list
    else:
        jobs_to_show = jobs

    for j in jobs_to_show:
        show_job(ofh, j)
        # add those jobs which are referenced by existing jobs, but which
        # might not be available as .conf files. For example, plymouth.conf
        # references gdm *or* kdm, but you are unlikely to have both
        # installed.
        for s in jobs[j]['start on']['job']:
            if s not in jobs_to_show:
                show_job(ofh, s)

        for s in jobs[j]['stop on']['job']:
            if s not in jobs_to_show:
                show_job(ofh, s)

    # Having displayed the jobs in restrictions_list,
    # we now need to display all jobs that *those* jobs
    # start on/stop on.
    for j in restrictions_list:
        for job in jobs[j]['start on']['job']:
            show_job(ofh, job)
        for job in jobs[j]['stop on']['job']:
            show_job(ofh, job)

    # Finally, show all jobs which emit events that jobs in the
    # restrictions_list care about.
    for j in restrictions_list:
        for e in jobs[j]['start on']['event']:
            for k in jobs:
                if e in jobs[k]['emits']:
                    show_job(ofh, k)

        for e in jobs[j]['stop on']['event']:
            for k in jobs:
                if e in jobs[k]['emits']:
                    show_job(ofh, k)


def show_edge(ofh, from_node, to_node, color):
    ofh.write("  %s -> %s [color=\"%s\"];\n" % (from_node, to_node, color))


def show_start_on_job_edge(ofh, from_job, to_job, relation):
    if relation == 'starting':
        show_edge(ofh, "%s:start" % mk_job_node_name(from_job),
                  "%s:job" % mk_job_node_name(to_job),options.color_start_on)
    else:
        show_edge(ofh, "%s:job" % mk_job_node_name(to_job),
                  "%s:start" % mk_job_node_name(from_job), options.color_start_on)


def show_start_on_event_edge(ofh, from_job, to_event):
    show_edge(ofh, mk_event_node_name(to_event),
              "%s:start" % mk_job_node_name(from_job), options.color_start_on)


def show_stop_on_job_edge(ofh, from_job, to_job):
    show_edge(ofh, "%s:job" % mk_job_node_name(to_job),
              "%s:stop" % mk_job_node_name(from_job), options.color_stop_on)


def show_stop_on_event_edge(ofh, from_job, to_event):
    show_edge(ofh, mk_event_node_name(to_event),
              "%s:stop" % mk_job_node_name(from_job), options.color_stop_on)


def show_job_emits_edge(ofh, from_job, to_event):
    show_edge(ofh, "%s:job" % mk_job_node_name(from_job),
              mk_event_node_name(to_event), options.color_emits)


def show_edges(ofh):
    glob_jobs = {}

    if restrictions_list:
        jobs_list = restrictions_list
    else:
        jobs_list = jobs

    for job in jobs_list:
        for s in jobs[job]['start on']['job']:
            show_start_on_job_edge(ofh, job, s, jobs[job]['start on']['job'][s])

        for s in jobs[job]['start on']['event']:
            show_start_on_event_edge(ofh, job, s)

        for s in jobs[job]['stop on']['job']:
            show_stop_on_job_edge(ofh, job, s)

        for s in jobs[job]['stop on']['event']:
            show_stop_on_event_edge(ofh, job, s)

        for e in jobs[job]['emits']:
            if '*' in e:
                # handle glob patterns in 'emits'
                glob_events = []
                for _e in events:
                    if e != _e and fnmatch.fnmatch(_e, e):
                        glob_events.append(_e)
                glob_jobs[job] = glob_events

            show_job_emits_edge(ofh, job, e)

        if not restrictions_list:
            continue

        # Add links to events emitted by all jobs which current job
        # start/stops on
        for j in jobs[job]['start on']['job']:
            if j not in jobs:
                continue
            for e in jobs[j]['emits']:
                show_job_emits_edge(ofh, j, e)

        for j in jobs[job]['stop on']['job']:
            for e in jobs[j]['emits']:
                show_job_emits_edge(ofh, j, e)

    # Create links from jobs (which advertise they emits a class of
    # events, via the glob syntax) to all the events they create.
    for g in glob_jobs:
        for ge in glob_jobs[g]:
            show_job_emits_edge(ofh, g, ge)

    if not restrictions_list:
        return

    # Add jobs->event links to jobs which emit events that current job
    # start/stops on.
    for j in restrictions_list:
        for e in jobs[j]['start on']['event']:
            for k in jobs:
                if e in jobs[k]['emits'] and e not in restrictions_list:
                    show_job_emits_edge(ofh, k, e)

        for e in jobs[j]['stop on']['event']:
            for k in jobs:
                if e in jobs[k]['emits'] and e not in restrictions_list:
                    show_job_emits_edge(ofh, k, e)


def read_data():
    global cmd
    global upstart_session
    global use_system

    if options.infile:
        try:
            ifh = open(options.infile, 'r')
        except:
            sys.exit("ERROR: cannot read file '%s'" % options.infile)
    else:
        try:
            ifh = Popen(cmd.split(), stdout=PIPE,
                        universal_newlines=True).stdout
        except:
            sys.exit("ERROR: cannot run '%s'" % cmd)

    job = None
    for line in ifh:
        line = line.rstrip()

        result = re.match('^\s+start on ([^,]+) \(job:\s*([^,]*), env:', line)
        if result:
            _event = encode_dollar(job, result.group(1))
            _job = result.group(2)
            if _job:
                jobs[job]['start on']['job'][_job] = _event
            else:
                jobs[job]['start on']['event'][_event] = 1
                events[_event] = 1
            continue

        result = re.match('^\s+stop on ([^,]+) \(job:\s*([^,]*), env:', line)
        if result:
            _event = encode_dollar(job, result.group(1))
            _job = result.group(2)
            if _job:
                jobs[job]['stop on']['job'][_job] = 1
            else:
                jobs[job]['stop on']['event'][_event] = 1
                events[_event] = 1
            continue

        if re.match('^\s+emits', line):
            event = line.lstrip().split()[1]
            event = encode_dollar(job, event)
            events[event] = 1
            jobs[job]['emits'][event] = 1
        else:
            tokens = line.lstrip().split()

            if len(tokens) != 1:
                sys.exit("ERROR: invalid line: %s" % line.lstrip())

            job_record = {}

            start_on = {}
            start_on_jobs = {}
            start_on_events = {}

            stop_on = {}
            stop_on_jobs = {}
            stop_on_events = {}

            emits = {}

            start_on['job'] = start_on_jobs
            start_on['event'] = start_on_events

            stop_on['job'] = stop_on_jobs
            stop_on['event'] = stop_on_events

            job_record['start on'] = start_on
            job_record['stop on'] = stop_on
            job_record['emits'] = emits

            job = (tokens)[0]
            jobs[job] = job_record


def main():
    global options
    global restrictions_list
    global cmd
    global use_system
    global upstart_session

    description = "Convert initctl(8) output to GraphViz dot(1) format."
    epilog = "See http://www.graphviz.org/doc/info/colors.html " \
             "for available colours."

    parser = ArgumentParser(description=description, epilog=epilog)

    parser.add_argument("-r", "--restrict-to-jobs",
                        dest="restrictions",
                        help="Limit display of 'start on' and 'stop on' "
                        "conditions to specified jobs (comma-separated list).")

    parser.add_argument("-f", "--infile",
                        dest="infile",
                        help="File to read output from. If not specified"
                        ", initctl will be run automatically.")

    parser.add_argument("-o", "--outfile",
                        dest="outfile",
                        help="File to write output to (default=%s)" %
                             default_outfile)

    parser.add_argument("--color-emits",
                        dest="color_emits",
                        help="Specify color for 'emits' lines (default=%s)." %
                             default_color_emits)

    parser.add_argument("--color-start-on",
                        dest="color_start_on",
                        help="Specify color for 'start on' lines "
                             "(default=%s)." % default_color_start_on)

    parser.add_argument("--color-stop-on",
                        dest="color_stop_on",
                        help="Specify color for 'stop on' lines "
                             "(default=%s)." % default_color_stop_on)

    parser.add_argument("--color-event",
                        dest="color_event",
                        help="Specify color for event boxes (default=%s)." %
                             default_color_event)

    parser.add_argument("--color-text",
                        dest="color_text",
                        help="Specify color for summary text (default=%s)." %
                             default_color_text)

    parser.add_argument("--color-bg",
                        dest="color_bg",
                        help="Specify background color for diagram "
                             "(default=%s)." % default_color_bg)

    parser.add_argument("--color-event-text",
                        dest="color_event_text",
                        help="Specify color for text in event boxes "
                             "(default=%s)." % default_color_text)

    parser.add_argument("--color-job-text",
                        dest="color_job_text",
                        help="Specify color for text in job boxes "
                             "(default=%s)." % default_color_text)

    parser.add_argument("--color-job",
                        dest="color_job",
                        help="Specify color for job boxes (default=%s)." %
                             default_color_job)

    parser.add_argument("--user",
                        dest="system",
                        default=None,
                        action='store_false',
                        help="Connect to Upstart user session (default if running within a user session).")

    parser.add_argument("--system",
                        dest="system",
                        default=None,
                        action='store_true',
                        help="Connect to Upstart system session.")

    parser.set_defaults(color_emits=default_color_emits,
                        color_start_on=default_color_start_on,
                        color_stop_on=default_color_stop_on,
                        color_event=default_color_event,
                        color_job=default_color_job,
                        color_job_text=default_color_text,
                        color_event_text=default_color_text,
                        color_text=default_color_text,
                        color_bg=default_color_bg,
                        outfile=default_outfile)

    options = parser.parse_args()

    if options.outfile == '-':
        ofh = sys.stdout
    else:
        try:
            ofh = open(options.outfile, "w")
        except:
            sys.exit("ERROR: cannot open file %s for writing" %
                     options.outfile)

    if options.restrictions:
        restrictions_list = options.restrictions.split(",")

    upstart_session = os.environ.get('UPSTART_SESSION', False)

    if options.system == None:
        if upstart_session:
            use_system = False
        else:
            use_system = True
    else:
        use_system = options.system or not upstart_session

    if use_system:
        cmd = "initctl --system show-config -e"
    else:
        cmd = "initctl show-config -e"

    read_data()

    for job in restrictions_list:
        if not job in jobs:
            sys.exit("ERROR: unknown job %s" % job)

    header(ofh)
    show_events(ofh)
    show_jobs(ofh)
    show_edges(ofh)
    footer(ofh)


if __name__ == "__main__":
    main()
