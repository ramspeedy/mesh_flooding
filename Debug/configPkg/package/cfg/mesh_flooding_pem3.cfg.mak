# invoke SourceDir generated makefile for mesh_flooding.pem3
mesh_flooding.pem3: .libraries,mesh_flooding.pem3
.libraries,mesh_flooding.pem3: package/cfg/mesh_flooding_pem3.xdl
	$(MAKE) -f /Users/rmenon/workspace_v7/mesh_flooding/src/makefile.libs

clean::
	$(MAKE) -f /Users/rmenon/workspace_v7/mesh_flooding/src/makefile.libs clean

