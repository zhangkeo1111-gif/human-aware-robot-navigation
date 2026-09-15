"""One official textured actor, driven by Isaac 6's native patrol/walking system."""
async def load_actor(stage, count=1, construction_actor=False, scenario=None):
    import carb
    import omni.kit.app
    import omni.kit.commands
    import omni.anim.navigation.core as nav
    from pxr import Gf, Sdf, UsdGeom
    from isaacsim.replicator.agent.core.configuration.models.character import CharacterConfig
    from isaacsim.replicator.agent.core.scene_assembly.character_loader import CharacterLoader
    from isaacsim.replicator.agent.core.randomizer import Randomizer

    # The shipped GPU baker failed with cudaErrorInvalidDevice on this machine.
    # CPU baking is an official setting; RTX rendering remains on the GPU.
    carb.settings.get_settings().set('/persistent/exts/omni.anim.navigation.core/navMesh/useGpu', False)
    omni.kit.commands.execute('CreateNavMeshVolumeCommand',
        parent_prim_path=Sdf.Path('/World'), volume_type=nav.NAVMESH_VOLUME_INCLUDE,
        position=Gf.Vec3d(3,0,0))
    await omni.kit.app.get_app().next_update_async()
    inav = nav.acquire_interface()
    inav.start_navmesh_baking_and_wait()
    if inav.get_navmesh() is None:
        raise RuntimeError('Actor navigation mesh did not bake')
    groups = {'pedestrian': {
        'num': 1, 'asset_path': 'Isaac/People/Characters/',
        'routines': [{'patrol': {'speed_range': [.65,.65],
            'path_points': [[3.,-1.,0.], [3.,1.,0.]], 'repeat': 100}}]}}
    if count >= 2:
        groups['second'] = {'num':1, 'asset_path':'Isaac/People/Characters/',
            'routines':[{'patrol':{'speed_range':[.55,.55],
                'path_points':[[4.5,-1.3,0.],[4.5,1.3,0.]], 'repeat':100}}]}
    if count == 3:
        groups['third']={'num':1,'asset_path':'Isaac/People/Characters/','routines':[]}
    if scenario == 'navwareset':
        groups['pedestrian']['asset_path']='Isaac/People/Characters/original_male_adult_medical_01/'
        if count >= 2:groups['second']['asset_path']='Isaac/People/Characters/original_male_adult_construction_03/'
    if scenario:
        # Scripted native walking is driven explicitly after initialization.
        # Identical scene/motion is used for no-avoidance and avoidance runs.
        for group in groups.values():
            group['routines'] = []
    config = CharacterConfig.model_validate({'groups': groups})
    class SelectedActorLoader(CharacterLoader):
        def _read_assets_from_directory(self, directory_url, file_extensions=None):
            from isaacsim.storage.native import get_assets_root_path
            return [get_assets_root_path()+'/Isaac/People/Characters/original_male_adult_construction_03/male_adult_construction_03.usd']
    class NavActorLoader(CharacterLoader):
        def _read_assets_from_directory(self,directory_url,file_extensions=None):
            if '/original_' in directory_url:
                asset=self._find_first_asset_in_folder(directory_url,file_extensions or ['usd','usda'])
                return [asset] if asset else []
            return [a for a in super()._read_assets_from_directory(directory_url,file_extensions)
                    if 'medical_01' not in a and 'construction_03' not in a]
    loader_class = NavActorLoader if scenario=='navwareset' else SelectedActorLoader if construction_actor else CharacterLoader
    loader = loader_class(config, Randomizer(17), stage)
    loaded = await loader.load()
    if loaded != count:
        raise RuntimeError(f'Expected {count} textured actors; loaded {loaded}')
    path = CharacterLoader.get_character_instance_prim_path(config.root_prim_path, 'pedestrian', 0)
    prim = stage.GetPrimAtPath(path)
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(3.,-1.,0.))
    if count >= 2:
        second = CharacterLoader.get_character_instance_prim_path(config.root_prim_path, 'second', 0)
        UsdGeom.XformCommonAPI(stage.GetPrimAtPath(second)).SetTranslate(Gf.Vec3d(4.5,1.3,0.))
    return path
