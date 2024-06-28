{% extends "_base.spec" %}

    {% block build %}
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
%py3_build
    {% endblock %}

    {% block install %}
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
%py3_install -- --prefix "{{ installation_prefix }}"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif
    {% endblock %}
