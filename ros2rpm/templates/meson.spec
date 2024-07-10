{% extends "_base.spec" %}

    {% block build %}
# override macro
%define __meson_auto_features auto
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
# call meson executable instead of using the 'meson' macro to use default paths
%__meson setup \
    --prefix="{{ installation_prefix }}" \
    --cmake-prefix-path="{{ installation_prefix }}" \
    --libdir=lib \
    --libexecdir=lib \
    %{_target_platform}
%meson_build -C %{_target_platform}
    {% endblock %}

    {% block install %}
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
%meson_install -C %{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__ninja -C %{_target_platform} -t targets | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "{{ installation_prefix }}/setup.sh" ]; then . "{{ installation_prefix }}/setup.sh"; fi
%meson_test -C %{_target_platform} || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif
    {% endblock %}
